#!/usr/bin/env python3
"""
Conveyor Robot Picking Simulator

This simulator intentionally contains no optimization logic. It only loads policy CSVs,
executes the recommended actions, applies the conveyor/robot rules, and exports metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import statistics
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

EMPTY_TOKENS = {"", "EMPTY", "NONE", "NULL", "-"}
NO_ACTION_TOKENS = {"", "NONE", "EMPTY", "NULL", "-"}


@dataclass
class RobotConfig:
    robot_id: str
    reachable_bins: List[int]
    pick_duration_timesteps: int = 1
    max_picks_per_timestep: int = 1


@dataclass
class SimulationConfig:
    num_bins: int
    item_types: List[str]
    robots: Dict[str, RobotConfig]
    pick_probabilities: Dict[str, Dict[str, float]]
    bin_move_timesteps: int = 1
    trials: int = 1
    random_seed: int = 42
    mode: str = "finite"  # finite or continuous
    max_timesteps: int = 1000
    initial_state: Dict[int, Dict[str, int]] = field(default_factory=dict)
    missing_state_behavior: str = "stop"  # stop or no_action
    invalid_action_behavior: str = "stop"  # stop or skip
    feed: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Policy:
    algorithm_name: str
    path: Path
    rows_by_state: Dict[str, Dict[str, str]]
    bin_columns: List[str]


@dataclass
class TrialResult:
    algorithm_name: str
    policy_csv_file: str
    trial_id: int
    random_seed: int
    num_bins: int
    num_robots: int
    elapsed_timesteps: int = 0
    total_successful_picks: int = 0
    total_failed_attempts: int = 0
    total_attempts: int = 0
    total_missed_items: int = 0
    missing_state_count: int = 0
    invalid_action_count: int = 0
    simulation_status: str = "complete"
    successful_picks_per_1000_timesteps: float = 0.0
    successful_picks_by_robot: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    failed_attempts_by_robot: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    successful_picks_by_type: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    missed_items_by_type: Dict[str, int] = field(default_factory=lambda: defaultdict(int))


class ConveyorState:
    """Stores bin contents as item counts. Bins are 1-indexed."""

    def __init__(self, num_bins: int, item_types: List[str], initial: Optional[Dict[int, Dict[str, int]]] = None):
        self.num_bins = num_bins
        self.item_types = list(item_types)
        self.bins: List[Dict[str, int]] = [defaultdict(int) for _ in range(num_bins + 1)]
        if initial:
            for bin_num, counts in initial.items():
                if 1 <= int(bin_num) <= num_bins:
                    for item, count in counts.items():
                        if count > 0:
                            self.bins[int(bin_num)][item] += int(count)

    def clone(self) -> "ConveyorState":
        copied = ConveyorState(self.num_bins, self.item_types)
        for i in range(1, self.num_bins + 1):
            copied.bins[i] = defaultdict(int, dict(self.bins[i]))
        return copied

    def has_items(self) -> bool:
        return any(sum(self.bins[i].values()) > 0 for i in range(1, self.num_bins + 1))

    def has_item(self, bin_num: int, item_type: str) -> bool:
        return 1 <= bin_num <= self.num_bins and self.bins[bin_num].get(item_type, 0) > 0

    def remove_one(self, bin_num: int, item_type: str) -> None:
        if not self.has_item(bin_num, item_type):
            raise ValueError(f"Cannot remove {item_type} from bin {bin_num}; item not present.")
        self.bins[bin_num][item_type] -= 1
        if self.bins[bin_num][item_type] <= 0:
            del self.bins[bin_num][item_type]

    def add_one(self, bin_num: int, item_type: str) -> None:
        if 1 <= bin_num <= self.num_bins:
            self.bins[bin_num][item_type] += 1

    def shift_forward(self) -> Dict[str, int]:
        """Shift items one bin forward. Return missed item counts."""
        missed = dict(self.bins[self.num_bins])
        for bin_num in range(self.num_bins, 1, -1):
            self.bins[bin_num] = self.bins[bin_num - 1]
        self.bins[1] = defaultdict(int)
        return missed

    def bin_string(self, bin_num: int) -> str:
        return counts_to_bin_string(self.bins[bin_num], self.item_types)

    def state_key(self) -> str:
        return "|".join(self.bin_string(i) for i in range(1, self.num_bins + 1))

    def row_dict(self) -> Dict[str, str]:
        return {f"bin_{i}": self.bin_string(i) for i in range(1, self.num_bins + 1)}



def counts_to_bin_string(counts: Dict[str, int], item_types: List[str]) -> str:
    if not counts:
        return "EMPTY"
    pieces: List[str] = []
    for item in sorted(counts.keys()):
        pieces.extend([item] * int(counts[item]))
    return "".join(pieces) if pieces else "EMPTY"


def parse_bin_cell(cell: Any, item_types: List[str]) -> Dict[str, int]:
    text = str(cell).strip().upper() if cell is not None else ""
    if text in EMPTY_TOKENS:
        return {}
    counts: Dict[str, int] = defaultdict(int)
    for ch in text:
        if ch.isspace() or ch == ",":
            continue
        if ch not in item_types:
            raise ValueError(f"Unknown item type '{ch}' in bin cell '{cell}'.")
        counts[ch] += 1
    return dict(counts)


def canonicalize_bin_cell(cell: Any, item_types: List[str]) -> str:
    return counts_to_bin_string(parse_bin_cell(cell, item_types), item_types)


def load_config(path: Path) -> SimulationConfig:
    with path.open("r", encoding="utf-8") as f:
        raw = json.load(f)

    robots: Dict[str, RobotConfig] = {}
    for robot_id, r in raw["robots"].items():
        robots[robot_id] = RobotConfig(
            robot_id=robot_id,
            reachable_bins=[int(x) for x in r["reachable_bins"]],
            pick_duration_timesteps=int(r.get("pick_duration_timesteps", 1)),
            max_picks_per_timestep=int(r.get("max_picks_per_timestep", 1)),
        )

    item_types = [str(x).upper() for x in raw["item_types"]]
    initial_state: Dict[int, Dict[str, int]] = {}
    for bin_key, content in raw.get("initial_state", {}).items():
        bin_num = int(str(bin_key).replace("bin_", ""))
        if isinstance(content, dict):
            initial_state[bin_num] = {str(k).upper(): int(v) for k, v in content.items() if int(v) > 0}
        elif isinstance(content, list):
            counts: Dict[str, int] = defaultdict(int)
            for item in content:
                item = str(item).upper()
                counts[item] += 1
            initial_state[bin_num] = dict(counts)
        else:
            initial_state[bin_num] = parse_bin_cell(content, item_types)

    cfg = SimulationConfig(
        num_bins=int(raw["num_bins"]),
        item_types=item_types,
        robots=robots,
        pick_probabilities={rid: {str(k).upper(): float(v) for k, v in probs.items()} for rid, probs in raw["pick_probabilities"].items()},
        bin_move_timesteps=int(raw.get("bin_move_timesteps", 1)),
        trials=int(raw.get("trials", 1)),
        random_seed=int(raw.get("random_seed", 42)),
        mode=str(raw.get("mode", "finite")).lower(),
        max_timesteps=int(raw.get("max_timesteps", 1000)),
        initial_state=initial_state,
        missing_state_behavior=str(raw.get("missing_state_behavior", "stop")).lower(),
        invalid_action_behavior=str(raw.get("invalid_action_behavior", "stop")).lower(),
        feed=raw.get("feed", {}),
    )
    validate_config(cfg)
    return cfg


def validate_config(cfg: SimulationConfig) -> None:
    if cfg.num_bins <= 0:
        raise ValueError("num_bins must be positive.")
    if cfg.bin_move_timesteps <= 0:
        raise ValueError("bin_move_timesteps must be positive.")
    if cfg.mode not in {"finite", "continuous"}:
        raise ValueError("mode must be 'finite' or 'continuous'.")
    for rid, robot in cfg.robots.items():
        if robot.pick_duration_timesteps <= 0:
            raise ValueError(f"{rid} pick_duration_timesteps must be positive.")
        for b in robot.reachable_bins:
            if b < 1 or b > cfg.num_bins:
                raise ValueError(f"{rid} reachable bin {b} is outside conveyor range.")
        if rid not in cfg.pick_probabilities:
            raise ValueError(f"Missing pick probabilities for {rid}.")
        for item in cfg.item_types:
            if item not in cfg.pick_probabilities[rid]:
                raise ValueError(f"Missing pick probability for {rid}/{item}.")
            p = cfg.pick_probabilities[rid][item]
            if p < 0 or p > 1:
                raise ValueError(f"Invalid pick probability for {rid}/{item}: {p}")


def load_policy(path: Path, algorithm_name: str, cfg: SimulationConfig) -> Policy:
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"Policy CSV {path} has no header.")
        fieldnames = set(reader.fieldnames)
        bin_columns = [f"bin_{i}" for i in range(1, cfg.num_bins + 1)]
        missing_bins = [c for c in bin_columns if c not in fieldnames]
        if missing_bins:
            raise ValueError(f"Policy CSV {path} missing bin columns: {missing_bins}")
        for rid in cfg.robots:
            for col in (f"{rid}_pick_bin", f"{rid}_pick_item"):
                if col not in fieldnames:
                    raise ValueError(f"Policy CSV {path} missing robot action column: {col}")

        rows_by_state: Dict[str, Dict[str, str]] = {}
        for row_number, row in enumerate(reader, start=2):
            state_key = "|".join(canonicalize_bin_cell(row.get(c, "EMPTY"), cfg.item_types) for c in bin_columns)
            if state_key in rows_by_state:
                raise ValueError(f"Duplicate state in {path} at row {row_number}: {state_key}")
            rows_by_state[state_key] = dict(row)
    return Policy(algorithm_name=algorithm_name, path=path, rows_by_state=rows_by_state, bin_columns=bin_columns)


def parse_policy_spec(spec: str) -> Tuple[str, Path]:
    if "=" in spec:
        name, path = spec.split("=", 1)
        return name.strip(), Path(path.strip())
    path = Path(spec)
    return path.stem, path


def robot_action_from_row(row: Dict[str, str], robot_id: str) -> Tuple[Optional[int], Optional[str]]:
    raw_bin = str(row.get(f"{robot_id}_pick_bin", "")).strip().upper()
    raw_item = str(row.get(f"{robot_id}_pick_item", "")).strip().upper()
    if raw_bin in NO_ACTION_TOKENS and raw_item in NO_ACTION_TOKENS:
        return None, None
    if raw_bin in NO_ACTION_TOKENS or raw_item in NO_ACTION_TOKENS:
        raise ValueError("Incomplete action: one of pick_bin or pick_item is NONE/EMPTY but the other is not.")
    try:
        return int(raw_bin), raw_item
    except ValueError as e:
        raise ValueError(f"pick_bin must be an integer or NONE, got '{raw_bin}'.") from e


def validate_action(state: ConveyorState, robot: RobotConfig, target_bin: Optional[int], item_type: Optional[str], cfg: SimulationConfig) -> Optional[str]:
    if target_bin is None and item_type is None:
        return None
    if target_bin is None or item_type is None:
        return "incomplete action"
    if target_bin not in robot.reachable_bins:
        return "target bin unreachable"
    if item_type not in cfg.item_types:
        return "unknown item type"
    if not state.has_item(target_bin, item_type):
        return "target item not present"
    return None


def robot_has_reachable_items(state: ConveyorState, robot: RobotConfig) -> bool:
    return any(sum(state.bins[b].values()) > 0 for b in robot.reachable_bins)


def maybe_inject_feed(state: ConveyorState, cfg: SimulationConfig, rng: random.Random) -> None:
    if cfg.mode != "continuous":
        return
    feed = cfg.feed or {}
    arrival_probability = float(feed.get("arrival_probability_per_movement", 0.0))
    max_items = int(feed.get("max_items_per_arrival", 1))
    if max_items <= 0:
        return
    if rng.random() > arrival_probability:
        return

    distribution = feed.get("item_type_distribution") or {item: 1.0 for item in cfg.item_types}
    items = list(distribution.keys())
    weights = [float(distribution[i]) for i in items]
    number_to_add = int(feed.get("items_per_arrival", 1))
    number_to_add = max(1, min(number_to_add, max_items))
    for _ in range(number_to_add):
        item = rng.choices(items, weights=weights, k=1)[0].upper()
        state.add_one(1, item)


def append_missing_state_record(records: List[Dict[str, Any]], policy: Policy, result: TrialResult, timestep: int, state: ConveyorState, reason: str) -> None:
    row = {
        "algorithm_name": policy.algorithm_name,
        "policy_csv_file": str(policy.path),
        "trial_id": result.trial_id,
        "timestep": timestep,
        "state_key": state.state_key(),
        "reason": reason,
    }
    row.update(state.row_dict())
    records.append(row)


def append_invalid_action_record(records: List[Dict[str, Any]], policy: Policy, result: TrialResult, timestep: int, state: ConveyorState, robot_id: str, target_bin: Any, item_type: Any, reason: str) -> None:
    row = {
        "algorithm_name": policy.algorithm_name,
        "policy_csv_file": str(policy.path),
        "trial_id": result.trial_id,
        "timestep": timestep,
        "state_key": state.state_key(),
        "robot_id": robot_id,
        "recommended_pick_bin": target_bin if target_bin is not None else "NONE",
        "recommended_pick_item": item_type if item_type is not None else "NONE",
        "reason": reason,
    }
    row.update(state.row_dict())
    records.append(row)


def run_trial(policy: Policy, cfg: SimulationConfig, trial_id: int, seed: int, missing_records: List[Dict[str, Any]], invalid_records: List[Dict[str, Any]]) -> TrialResult:
    rng = random.Random(seed)
    state = ConveyorState(cfg.num_bins, cfg.item_types, cfg.initial_state)
    result = TrialResult(
        algorithm_name=policy.algorithm_name,
        policy_csv_file=str(policy.path),
        trial_id=trial_id,
        random_seed=seed,
        num_bins=cfg.num_bins,
        num_robots=len(cfg.robots),
    )
    robot_busy_until = {rid: 0 for rid in cfg.robots}

    for timestep in range(cfg.max_timesteps):
        if cfg.mode == "finite" and not state.has_items():
            result.simulation_status = "complete"
            break

        state_key = state.state_key()
        row = policy.rows_by_state.get(state_key)
        if row is None:
            result.missing_state_count += 1
            append_missing_state_record(missing_records, policy, result, timestep, state, "state not found in policy CSV")
            if cfg.missing_state_behavior == "stop":
                result.simulation_status = "stopped_missing_state"
                result.elapsed_timesteps = timestep
                break
            row = {}

        # Execute policy actions for available robots. Busy robots are skipped, not considered invalid.
        for rid, robot in cfg.robots.items():
            if timestep < robot_busy_until[rid]:
                continue
            if not robot_has_reachable_items(state, robot):
                continue

            try:
                target_bin, item_type = robot_action_from_row(row, rid) if row else (None, None)
            except ValueError as e:
                result.invalid_action_count += 1
                append_invalid_action_record(invalid_records, policy, result, timestep, state, rid, "MALFORMED", "MALFORMED", str(e))
                if cfg.invalid_action_behavior == "stop":
                    result.simulation_status = "stopped_invalid_action"
                    result.elapsed_timesteps = timestep
                    finalize_result(result)
                    return result
                continue

            invalid_reason = validate_action(state, robot, target_bin, item_type, cfg)
            if invalid_reason:
                result.invalid_action_count += 1
                append_invalid_action_record(invalid_records, policy, result, timestep, state, rid, target_bin, item_type, invalid_reason)
                if cfg.invalid_action_behavior == "stop":
                    result.simulation_status = "stopped_invalid_action"
                    result.elapsed_timesteps = timestep
                    finalize_result(result)
                    return result
                continue

            if target_bin is None and item_type is None:
                continue

            # The pick outcome resolves immediately. pick_duration_timesteps controls how long the robot stays busy afterward.
            result.total_attempts += 1
            p_success = cfg.pick_probabilities[rid][item_type]
            if rng.random() < p_success:
                state.remove_one(target_bin, item_type)
                result.total_successful_picks += 1
                result.successful_picks_by_robot[rid] += 1
                result.successful_picks_by_type[item_type] += 1
            else:
                result.total_failed_attempts += 1
                result.failed_attempts_by_robot[rid] += 1
            robot_busy_until[rid] = timestep + robot.pick_duration_timesteps

        # Conveyor moves after actions on configured movement ticks.
        if (timestep + 1) % cfg.bin_move_timesteps == 0:
            missed = state.shift_forward()
            for item, count in missed.items():
                result.total_missed_items += int(count)
                result.missed_items_by_type[item] += int(count)
            maybe_inject_feed(state, cfg, rng)

        result.elapsed_timesteps = timestep + 1
    else:
        result.simulation_status = "max_timesteps_reached"

    finalize_result(result)
    return result


def finalize_result(result: TrialResult) -> None:
    if result.elapsed_timesteps > 0:
        result.successful_picks_per_1000_timesteps = result.total_successful_picks / result.elapsed_timesteps * 1000.0
    else:
        result.successful_picks_per_1000_timesteps = 0.0


def result_to_row(result: TrialResult, cfg: SimulationConfig) -> Dict[str, Any]:
    row: Dict[str, Any] = {
        "algorithm_name": result.algorithm_name,
        "policy_csv_file": result.policy_csv_file,
        "trial_id": result.trial_id,
        "random_seed": result.random_seed,
        "num_bins": result.num_bins,
        "num_robots": result.num_robots,
        "elapsed_timesteps": result.elapsed_timesteps,
        "total_successful_picks": result.total_successful_picks,
        "total_failed_attempts": result.total_failed_attempts,
        "total_attempts": result.total_attempts,
        "total_missed_items": result.total_missed_items,
        "successful_picks_per_1000_timesteps": f"{result.successful_picks_per_1000_timesteps:.6f}",
        "missing_state_count": result.missing_state_count,
        "invalid_action_count": result.invalid_action_count,
        "simulation_status": result.simulation_status,
    }
    for rid in cfg.robots:
        row[f"{rid}_successful_picks"] = result.successful_picks_by_robot.get(rid, 0)
        row[f"{rid}_failed_attempts"] = result.failed_attempts_by_robot.get(rid, 0)
    for item in cfg.item_types:
        row[f"{item}_successful_picks"] = result.successful_picks_by_type.get(item, 0)
        row[f"{item}_missed_items"] = result.missed_items_by_type.get(item, 0)
    return row


def write_csv(path: Path, rows: List[Dict[str, Any]], fieldnames: Optional[List[str]] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        keys: List[str] = []
        seen = set()
        for row in rows:
            for key in row.keys():
                if key not in seen:
                    seen.add(key)
                    keys.append(key)
        fieldnames = keys
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def make_summary_rows(results: List[TrialResult]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[TrialResult]] = defaultdict(list)
    for r in results:
        grouped[r.algorithm_name].append(r)
    rows: List[Dict[str, Any]] = []
    for algorithm_name, group in grouped.items():
        def avg(attr: str) -> float:
            return statistics.mean(getattr(r, attr) for r in group) if group else 0.0
        rows.append({
            "algorithm_name": algorithm_name,
            "num_trials": len(group),
            "completed_trials": sum(1 for r in group if r.simulation_status == "complete"),
            "avg_elapsed_timesteps": f"{avg('elapsed_timesteps'):.6f}",
            "avg_successful_picks": f"{avg('total_successful_picks'):.6f}",
            "avg_failed_attempts": f"{avg('total_failed_attempts'):.6f}",
            "avg_total_attempts": f"{avg('total_attempts'):.6f}",
            "avg_missed_items": f"{avg('total_missed_items'):.6f}",
            "avg_successful_picks_per_1000_timesteps": f"{avg('successful_picks_per_1000_timesteps'):.6f}",
            "missing_state_count": sum(r.missing_state_count for r in group),
            "invalid_action_count": sum(r.invalid_action_count for r in group),
        })
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Run conveyor robot policy simulations.")
    parser.add_argument("--config", required=True, help="Path to simulation config JSON.")
    parser.add_argument("--policy", action="append", required=True, help="Policy CSV path, or name=path. May be repeated.")
    parser.add_argument("--output-dir", default="simulation_outputs", help="Directory for output CSV files.")
    args = parser.parse_args()

    cfg = load_config(Path(args.config))
    policies = []
    for spec in args.policy:
        name, path = parse_policy_spec(spec)
        policies.append(load_policy(path, name, cfg))

    output_dir = Path(args.output_dir)
    all_results: List[TrialResult] = []
    missing_records: List[Dict[str, Any]] = []
    invalid_records: List[Dict[str, Any]] = []

    for policy in policies:
        for trial_id in range(1, cfg.trials + 1):
            seed = cfg.random_seed + trial_id - 1
            result = run_trial(policy, cfg, trial_id, seed, missing_records, invalid_records)
            all_results.append(result)

    result_rows = [result_to_row(r, cfg) for r in all_results]
    write_csv(output_dir / "run_results.csv", result_rows)
    write_csv(output_dir / "summary_results.csv", make_summary_rows(all_results))

    missing_fieldnames = ["algorithm_name", "policy_csv_file", "trial_id", "timestep", "state_key"] + [f"bin_{i}" for i in range(1, cfg.num_bins + 1)] + ["reason"]
    invalid_fieldnames = ["algorithm_name", "policy_csv_file", "trial_id", "timestep", "state_key", "robot_id", "recommended_pick_bin", "recommended_pick_item"] + [f"bin_{i}" for i in range(1, cfg.num_bins + 1)] + ["reason"]
    write_csv(output_dir / "missing_states.csv", missing_records, missing_fieldnames)
    write_csv(output_dir / "invalid_actions.csv", invalid_records, invalid_fieldnames)

    print(f"Wrote results to: {output_dir.resolve()}")
    print(f"- {output_dir / 'run_results.csv'}")
    print(f"- {output_dir / 'summary_results.csv'}")
    print(f"- {output_dir / 'missing_states.csv'}")
    print(f"- {output_dir / 'invalid_actions.csv'}")


if __name__ == "__main__":
    main()
