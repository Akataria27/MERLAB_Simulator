#!/usr/bin/env python3
"""
Recycling conveyor-belt robot picking simulator.

The simulator is a referee only. It loads policy definitions from config,
runs finite-mode episodes, simulates pick outcomes, and writes metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

EMPTY = "EMPTY"
NONE = "NONE"


class ConfigError(Exception):
    pass


@dataclass(frozen=True)
class RobotConfig:
    name: str
    reachable_bins: List[int]
    pick_duration_timesteps: int = 1


@dataclass(frozen=True)
class Action:
    robot_name: str
    pick_bin: Optional[int]
    pick_item: Optional[str]

    @property
    def is_none(self) -> bool:
        return self.pick_bin is None or self.pick_item is None


@dataclass
class RobotRuntime:
    config: RobotConfig
    busy_until_timestep: int = 0

    def available(self, timestep: int) -> bool:
        return timestep >= self.busy_until_timestep


@dataclass
class TrialMetrics:
    algorithm_name: str
    policy_type: str
    policy_csv_file: str
    trial_id: int
    random_seed: int
    num_bins: int
    num_robots: int
    initial_state_id: str
    elapsed_timesteps: int = 0
    total_successful_picks: int = 0
    total_failed_attempts: int = 0
    total_attempts: int = 0
    total_missed_items: int = 0
    missing_state_count: int = 0
    simulation_status: str = "completed"

    @property
    def successful_picks_per_1000_timesteps(self) -> float:
        if self.elapsed_timesteps <= 0:
            return 0.0
        return self.total_successful_picks / self.elapsed_timesteps * 1000.0


@dataclass
class MissingStateRecord:
    algorithm_name: str
    policy_csv_file: str
    trial_id: int
    timestep: int
    state_key: str
    bins: List[str]
    robot_busy_status: str
    reason: str


class Policy:
    name: str
    policy_type: str
    policy_csv_file: str = ""

    def choose_actions(
        self,
        bins: List[str],
        robots: Dict[str, RobotRuntime],
        timestep: int,
        context: "SimulationContext",
    ) -> Tuple[List[Action], bool]:
        raise NotImplementedError


class CsvPolicy(Policy):
    def __init__(self, name: str, file_path: Path, num_bins: int, robots: Sequence[RobotConfig]):
        self.name = name
        self.policy_type = "csv"
        self.policy_csv_file = str(file_path)
        self.num_bins = num_bins
        self.robot_names = [r.name for r in robots]
        self.rows_by_state: Dict[Tuple[str, ...], Dict[str, str]] = {}
        self._load(file_path)

    def _load(self, file_path: Path) -> None:
        if not file_path.exists():
            raise ConfigError(f"Policy CSV not found: {file_path}")

        with file_path.open("r", newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise ConfigError(f"Policy CSV has no header: {file_path}")

            required = ["state_id"] + [f"bin_{i}" for i in range(1, self.num_bins + 1)]
            for robot_name in self.robot_names:
                required += [f"{robot_name}_pick_bin", f"{robot_name}_pick_item"]

            missing = [c for c in required if c not in reader.fieldnames]
            if missing:
                raise ConfigError(
                    f"Policy CSV {file_path} is missing required columns: {', '.join(missing)}"
                )

            for row in reader:
                key = tuple(canonicalize_cell(row.get(f"bin_{i}", EMPTY)) for i in range(1, self.num_bins + 1))
                self.rows_by_state[key] = row

    def choose_actions(
        self,
        bins: List[str],
        robots: Dict[str, RobotRuntime],
        timestep: int,
        context: "SimulationContext",
    ) -> Tuple[List[Action], bool]:
        key = tuple(bins)
        row = self.rows_by_state.get(key)
        if row is None:
            return [], True

        actions: List[Action] = []
        for robot_name, robot in robots.items():
            if not robot.available(timestep):
                continue
            raw_bin = normalize_token(row.get(f"{robot_name}_pick_bin", NONE))
            raw_item = normalize_token(row.get(f"{robot_name}_pick_item", NONE))
            if raw_bin == NONE or raw_item == NONE or raw_bin == "":
                actions.append(Action(robot_name, None, None))
                continue
            try:
                pick_bin = int(float(raw_bin))
            except ValueError:
                # Malformed policy action is treated as no action by design.
                actions.append(Action(robot_name, None, None))
                continue
            actions.append(Action(robot_name, pick_bin, raw_item))
        return actions, False


class RandomPolicy(Policy):
    def __init__(self, name: str):
        self.name = name
        self.policy_type = "random"
        self.policy_csv_file = ""

    def choose_actions(
        self,
        bins: List[str],
        robots: Dict[str, RobotRuntime],
        timestep: int,
        context: "SimulationContext",
    ) -> Tuple[List[Action], bool]:
        actions: List[Action] = []
        for robot_name, robot in robots.items():
            if not robot.available(timestep):
                continue
            candidates = valid_pick_candidates(bins, robot.config)
            if not candidates:
                actions.append(Action(robot_name, None, None))
            else:
                pick_bin, pick_item = context.rng.choice(candidates)
                actions.append(Action(robot_name, pick_bin, pick_item))
        return actions, False


class HighestProbabilityPolicy(Policy):
    def __init__(self, name: str):
        self.name = name
        self.policy_type = "highest_probability"
        self.policy_csv_file = ""

    def choose_actions(
        self,
        bins: List[str],
        robots: Dict[str, RobotRuntime],
        timestep: int,
        context: "SimulationContext",
    ) -> Tuple[List[Action], bool]:
        actions: List[Action] = []
        for robot_name, robot in robots.items():
            if not robot.available(timestep):
                continue
            candidates = valid_pick_candidates(bins, robot.config)
            if not candidates:
                actions.append(Action(robot_name, None, None))
                continue
            probs = context.pick_probabilities.get(robot_name, {})
            max_prob = max(float(probs.get(item, 0.0)) for _, item in candidates)
            best = [(b, item) for b, item in candidates if float(probs.get(item, 0.0)) == max_prob]
            pick_bin, pick_item = context.rng.choice(best)
            actions.append(Action(robot_name, pick_bin, pick_item))
        return actions, False


@dataclass
class SimulationContext:
    config: Dict[str, Any]
    rng: random.Random
    pick_probabilities: Dict[str, Dict[str, float]]


def normalize_token(value: Any) -> str:
    if value is None:
        return ""
    return str(value).strip().upper()


def canonicalize_cell(value: Any) -> str:
    token = normalize_token(value)
    if token in ("", EMPTY, NONE):
        return EMPTY
    chars = [ch for ch in token if not ch.isspace()]
    return "".join(sorted(chars)) if chars else EMPTY


def state_key(bins: Sequence[str]) -> str:
    return "|".join(bins)


def count_items(bins: Sequence[str]) -> int:
    return sum(0 if cell == EMPTY else len(cell) for cell in bins)


def valid_pick_candidates(bins: Sequence[str], robot: RobotConfig) -> List[Tuple[int, str]]:
    candidates: List[Tuple[int, str]] = []
    for bin_num in robot.reachable_bins:
        if bin_num < 1 or bin_num > len(bins):
            continue
        cell = bins[bin_num - 1]
        if cell == EMPTY:
            continue
        for item in sorted(set(cell)):
            candidates.append((bin_num, item))
    return candidates


def remove_one_item(cell: str, item: str) -> str:
    if cell == EMPTY:
        return EMPTY
    idx = cell.find(item)
    if idx < 0:
        return cell
    new_cell = cell[:idx] + cell[idx + 1 :]
    return canonicalize_cell(new_cell)


def move_belt_forward(bins: List[str]) -> int:
    missed = 0 if bins[-1] == EMPTY else len(bins[-1])
    for i in range(len(bins) - 1, 0, -1):
        bins[i] = bins[i - 1]
    bins[0] = EMPTY
    return missed


def robot_busy_status(robots: Dict[str, RobotRuntime], timestep: int) -> str:
    parts = []
    for name, robot in robots.items():
        if robot.available(timestep):
            parts.append(f"{name}:available")
        else:
            parts.append(f"{name}:busy_until_{robot.busy_until_timestep}")
    return ";".join(parts)


def make_initial_state(config: Dict[str, Any], rng: random.Random) -> List[str]:
    num_bins = int(config["num_bins"])
    bins = [EMPTY for _ in range(num_bins)]

    random_initial = config.get("random_initial_state", {})
    if random_initial.get("enabled", False):
        target_bins = [int(b) for b in random_initial.get("bins", [1, 2, 3])]
        min_total = int(random_initial.get("min_total_items", 0))
        max_total = int(random_initial.get("max_total_items", 5))
        if min_total < 0 or max_total < min_total:
            raise ConfigError("Invalid random_initial_state min/max total item range.")
        total_items = rng.randint(min_total, max_total)

        distribution = random_initial.get("item_type_distribution", config.get("item_types", {}))
        if isinstance(distribution, dict):
            item_types = list(distribution.keys())
            weights = [float(v) for v in distribution.values()]
        else:
            item_types = list(distribution)
            weights = [1.0] * len(item_types)
        if not item_types or sum(weights) <= 0:
            raise ConfigError("Random initial state needs at least one item type with positive weight.")

        for _ in range(total_items):
            bin_num = rng.choice(target_bins)
            item = rng.choices(item_types, weights=weights, k=1)[0]
            bins[bin_num - 1] = canonicalize_cell(("" if bins[bin_num - 1] == EMPTY else bins[bin_num - 1]) + item)
        return bins

    initial_state = config.get("initial_state", {})
    for key, value in initial_state.items():
        if isinstance(key, str) and key.startswith("bin_"):
            bin_num = int(key.split("_", 1)[1])
        else:
            bin_num = int(key)
        bins[bin_num - 1] = canonicalize_cell(value)
    return bins


def load_config(config_path: Path) -> Dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as f:
        config = json.load(f)

    required_top = ["num_bins", "robots", "pick_probabilities", "policies"]
    missing = [k for k in required_top if k not in config]
    if missing:
        raise ConfigError(f"Config missing required keys: {', '.join(missing)}")

    config.setdefault("mode", "finite")
    config.setdefault("trials", 1000)
    config.setdefault("bin_move_timesteps", 1)
    config.setdefault("outputs", {})
    config["outputs"].setdefault("base_dir", "outputs")
    config["outputs"].setdefault("write_run_results", False)
    return config


def load_robots(config: Dict[str, Any]) -> Dict[str, RobotConfig]:
    robots: Dict[str, RobotConfig] = {}
    for entry in config["robots"]:
        name = entry["name"]
        robots[name] = RobotConfig(
            name=name,
            reachable_bins=[int(b) for b in entry["reachable_bins"]],
            pick_duration_timesteps=int(entry.get("pick_duration_timesteps", config.get("default_pick_duration_timesteps", 1))),
        )
    return robots


def load_policies(config: Dict[str, Any], config_path: Path, robots: Sequence[RobotConfig]) -> List[Policy]:
    policies: List[Policy] = []
    base = config_path.parent
    for entry in config["policies"]:
        name = entry["name"]
        typ = entry["type"]
        if typ == "csv":
            file_path = Path(entry["file"])
            if not file_path.is_absolute():
                file_path = base / file_path
            policies.append(CsvPolicy(name, file_path, int(config["num_bins"]), robots))
        elif typ == "random":
            policies.append(RandomPolicy(name))
        elif typ == "highest_probability":
            policies.append(HighestProbabilityPolicy(name))
        else:
            raise ConfigError(f"Unsupported policy type for {name}: {typ}")
    return policies


def apply_actions(
    actions: Sequence[Action],
    bins: List[str],
    robots: Dict[str, RobotRuntime],
    timestep: int,
    context: SimulationContext,
    metrics: TrialMetrics,
) -> None:
    # Robot reach zones are assumed non-overlapping. Actions are processed in config order.
    for action in actions:
        if action.is_none:
            continue
        robot = robots[action.robot_name]
        if not robot.available(timestep):
            continue
        if action.pick_bin not in robot.config.reachable_bins:
            continue
        if action.pick_bin is None or action.pick_bin < 1 or action.pick_bin > len(bins):
            continue
        item = normalize_token(action.pick_item)
        if item == NONE or item == "":
            continue
        cell = bins[action.pick_bin - 1]
        if cell == EMPTY or item not in cell:
            continue

        metrics.total_attempts += 1
        prob = float(context.pick_probabilities.get(action.robot_name, {}).get(item, 0.0))
        if context.rng.random() < prob:
            bins[action.pick_bin - 1] = remove_one_item(cell, item)
            metrics.total_successful_picks += 1
        else:
            metrics.total_failed_attempts += 1
        robot.busy_until_timestep = timestep + int(robot.config.pick_duration_timesteps)


def run_trial(
    policy: Policy,
    trial_id: int,
    seed: int,
    config: Dict[str, Any],
    robot_configs: Dict[str, RobotConfig],
    initial_seed: Optional[int] = None,
) -> Tuple[TrialMetrics, List[MissingStateRecord]]:
    rng = random.Random(seed)
    initial_rng = random.Random(seed if initial_seed is None else initial_seed)
    context = SimulationContext(config=config, rng=rng, pick_probabilities=config["pick_probabilities"])
    bins = make_initial_state(config, initial_rng)
    initial_state_id = state_key(bins)
    robots = {name: RobotRuntime(cfg) for name, cfg in robot_configs.items()}
    metrics = TrialMetrics(
        algorithm_name=policy.name,
        policy_type=policy.policy_type,
        policy_csv_file=getattr(policy, "policy_csv_file", ""),
        trial_id=trial_id,
        random_seed=seed,
        num_bins=int(config["num_bins"]),
        num_robots=len(robot_configs),
        initial_state_id=initial_state_id,
    )
    missing_records: List[MissingStateRecord] = []

    timestep = 0
    bin_move_timesteps = int(config.get("bin_move_timesteps", 1))
    if bin_move_timesteps <= 0:
        raise ConfigError("bin_move_timesteps must be positive.")

    while count_items(bins) > 0:
        actions, missing = policy.choose_actions(bins, robots, timestep, context)
        if missing:
            metrics.missing_state_count += 1
            missing_records.append(
                MissingStateRecord(
                    algorithm_name=policy.name,
                    policy_csv_file=getattr(policy, "policy_csv_file", ""),
                    trial_id=trial_id,
                    timestep=timestep,
                    state_key=state_key(bins),
                    bins=list(bins),
                    robot_busy_status=robot_busy_status(robots, timestep),
                    reason="state_not_found_in_policy_csv",
                )
            )
        else:
            apply_actions(actions, bins, robots, timestep, context, metrics)

        timestep += 1
        if timestep % bin_move_timesteps == 0:
            metrics.total_missed_items += move_belt_forward(bins)

    metrics.elapsed_timesteps = timestep
    if metrics.missing_state_count > 0:
        metrics.simulation_status = "completed_with_missing_states"
    return metrics, missing_records


def safe_title(title: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", title.strip())
    cleaned = cleaned.strip("._-")
    if not cleaned:
        raise ConfigError("Run title must contain at least one letter or number.")
    return cleaned


def create_output_dir(config: Dict[str, Any], config_path: Path, run_title: str) -> Path:
    base_dir = Path(config.get("outputs", {}).get("base_dir", "outputs"))
    if not base_dir.is_absolute():
        base_dir = config_path.parent / base_dir
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = base_dir / f"{safe_title(run_title)}_{timestamp}"
    out_dir.mkdir(parents=True, exist_ok=False)
    return out_dir


def write_missing_states(path: Path, records: Sequence[MissingStateRecord], num_bins: int) -> None:
    fields = [
        "algorithm_name",
        "policy_csv_file",
        "trial_id",
        "timestep",
        "state_key",
    ] + [f"bin_{i}" for i in range(1, num_bins + 1)] + ["robot_busy_status", "reason"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for rec in records:
            row = {
                "algorithm_name": rec.algorithm_name,
                "policy_csv_file": rec.policy_csv_file,
                "trial_id": rec.trial_id,
                "timestep": rec.timestep,
                "state_key": rec.state_key,
                "robot_busy_status": rec.robot_busy_status,
                "reason": rec.reason,
            }
            for i, cell in enumerate(rec.bins, start=1):
                row[f"bin_{i}"] = cell
            writer.writerow(row)


def write_run_results(path: Path, metrics_rows: Sequence[TrialMetrics]) -> None:
    fields = [
        "algorithm_name",
        "policy_type",
        "policy_csv_file",
        "trial_id",
        "random_seed",
        "num_bins",
        "num_robots",
        "initial_state_id",
        "elapsed_timesteps",
        "total_successful_picks",
        "total_failed_attempts",
        "total_attempts",
        "total_missed_items",
        "successful_picks_per_1000_timesteps",
        "missing_state_count",
        "simulation_status",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for m in metrics_rows:
            writer.writerow({field: getattr(m, field) if hasattr(m, field) else None for field in fields})


def average(values: Iterable[float]) -> float:
    vals = list(values)
    return sum(vals) / len(vals) if vals else 0.0


def write_summary(path: Path, metrics_rows: Sequence[TrialMetrics]) -> None:
    by_alg: Dict[str, List[TrialMetrics]] = {}
    for m in metrics_rows:
        by_alg.setdefault(m.algorithm_name, []).append(m)

    fields = [
        "algorithm_name",
        "policy_type",
        "policy_csv_file",
        "num_trials",
        "completed_trials",
        "completed_with_missing_state_trials",
        "avg_successful_picks",
        "avg_failed_attempts",
        "avg_missed_items",
        "avg_total_attempts",
        "avg_elapsed_timesteps",
        "avg_successful_picks_per_1000_timesteps",
        "total_missing_states",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for alg, rows in by_alg.items():
            writer.writerow(
                {
                    "algorithm_name": alg,
                    "policy_type": rows[0].policy_type,
                    "policy_csv_file": rows[0].policy_csv_file,
                    "num_trials": len(rows),
                    "completed_trials": sum(1 for r in rows if r.simulation_status == "completed"),
                    "completed_with_missing_state_trials": sum(
                        1 for r in rows if r.simulation_status == "completed_with_missing_states"
                    ),
                    "avg_successful_picks": average(r.total_successful_picks for r in rows),
                    "avg_failed_attempts": average(r.total_failed_attempts for r in rows),
                    "avg_missed_items": average(r.total_missed_items for r in rows),
                    "avg_total_attempts": average(r.total_attempts for r in rows),
                    "avg_elapsed_timesteps": average(r.elapsed_timesteps for r in rows),
                    "avg_successful_picks_per_1000_timesteps": average(
                        r.successful_picks_per_1000_timesteps for r in rows
                    ),
                    "total_missing_states": sum(r.missing_state_count for r in rows),
                }
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run finite-mode recycling policy simulations.")
    parser.add_argument("--config", required=True, help="Path to JSON config file.")
    parser.add_argument("--run-title", required=True, help="Name prefix for the timestamped output folder.")
    parser.add_argument("--trials", type=int, default=None, help="Override number of trials in config.")
    parser.add_argument("--seed", type=int, default=None, help="Override base random seed in config.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    config = load_config(config_path)
    if config.get("mode", "finite") != "finite":
        raise ConfigError("Only finite mode is implemented. Continuous mode is intentionally blank for now.")

    trials = int(args.trials if args.trials is not None else config.get("trials", 1000))
    if trials < 1:
        raise ConfigError("Trials must be at least 1.")
    base_seed = int(args.seed if args.seed is not None else config.get("random_seed", 1))

    robot_configs = load_robots(config)
    policies = load_policies(config, config_path, list(robot_configs.values()))
    out_dir = create_output_dir(config, config_path, args.run_title)

    all_metrics: List[TrialMetrics] = []
    all_missing: List[MissingStateRecord] = []

    for policy_index, policy in enumerate(policies):
        for trial_id in range(1, trials + 1):
            # Same trial_id gets same initial randomness across policies, offset by policy only for pick outcomes.
            initial_seed = base_seed + trial_id * 1000003
            seed = initial_seed + policy_index * 1009
            metrics, missing = run_trial(policy, trial_id, seed, config, robot_configs, initial_seed=initial_seed)
            all_metrics.append(metrics)
            all_missing.extend(missing)

    write_summary(out_dir / "summary_results.csv", all_metrics)
    write_missing_states(out_dir / "missing_states.csv", all_missing, int(config["num_bins"]))
    if bool(config.get("outputs", {}).get("write_run_results", False)):
        write_run_results(out_dir / "run_results.csv", all_metrics)

    print(f"Simulation complete. Output folder: {out_dir}")
    print(f"Policies run: {len(policies)}")
    print(f"Trials per policy: {trials}")


if __name__ == "__main__":
    try:
        main()
    except ConfigError as e:
        raise SystemExit(f"Config error: {e}")
