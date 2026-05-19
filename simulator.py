# Conveyor Robot Picking Simulator

This project simulates a conveyor belt with robot pickers and scores externally generated policy CSV files. The simulator is intentionally separate from optimization: it does not search for better actions, compute optimal actions, train a model, or modify the policy. It only executes the actions supplied in a policy CSV and records what happened.

## What the simulator does

- Loads a shared JSON simulation config.
- Loads one or more algorithm policy CSV files.
- Simulates conveyor movement, robot availability, probabilistic pick success, failed attempts, missed items, and optional continuous item feed.
- Supports a variable number of bins.
- Supports a variable number of robots.
- Supports variable conveyor movement timing.
- Supports variable robot pick duration timing.
- Exports results as CSV files.
- Records missing policy states so they can be debugged.
- Records invalid policy actions so they can be debugged.

## What the simulator does not do

The simulator does **not** optimize. It does not choose the best action, calculate expected value, repair a policy table, or use a fallback optimizer. Algorithm code should generate the policy CSV separately.

The clean split is:

```text
Algorithm / optimizer -> creates policy CSV
Simulator             -> executes policy CSV and scores it
```

## Files

```text
simulator.py          Main simulator script
config.example.json   Example simulation configuration
policy.example.csv    Tiny example policy CSV
README.md             This file
```

## Requirements

Python 3.9 or newer is recommended. The script only uses the Python standard library, so no external packages are required.

## VS Code setup

1. Open the folder containing these files in VS Code.
2. Open a terminal in VS Code:

```text
Terminal -> New Terminal
```

3. Check Python is available:

```bash
python --version
```

On some systems, use:

```bash
python3 --version
```

4. Run the simulator from the folder containing `simulator.py`.

## Basic run command

Windows PowerShell:

```powershell
python simulator.py --config config.example.json --policy example=policy.example.csv --output-dir outputs
```

macOS/Linux:

```bash
python3 simulator.py --config config.example.json --policy example=policy.example.csv --output-dir outputs
```

After running, output CSVs will be written to the selected output directory.

## Running multiple algorithms

Each algorithm should provide its own policy CSV. The simulator can run several policy files in one command:

```bash
python3 simulator.py \
  --config config.example.json \
  --policy dp=dp_policy.csv \
  --policy greedy=greedy_policy.csv \
  --policy heuristic=heuristic_policy.csv \
  --output-dir outputs
```

On Windows PowerShell, use backticks for line continuation:

```powershell
python simulator.py `
  --config config.example.json `
  --policy dp=dp_policy.csv `
  --policy greedy=greedy_policy.csv `
  --policy heuristic=heuristic_policy.csv `
  --output-dir outputs
```

## Config file

The simulator uses a JSON config file. Important fields:

```json
{
  "num_bins": 15,
  "item_types": ["A", "B", "C", "D", "E"],
  "bin_move_timesteps": 1,
  "trials": 10,
  "random_seed": 42,
  "mode": "finite",
  "max_timesteps": 1000,
  "missing_state_behavior": "stop",
  "invalid_action_behavior": "stop"
}
```

### Conveyor timing

```json
"bin_move_timesteps": 1
```

This means the belt advances one bin every timestep.

```json
"bin_move_timesteps": 3
```

This means the belt advances one bin every 3 timesteps.

### Robot timing

Each robot has its own pick duration:

```json
"R1": {
  "reachable_bins": [4, 5, 6],
  "pick_duration_timesteps": 2,
  "max_picks_per_timestep": 1
}
```

In the current implementation, the pick success/failure is resolved immediately when the robot starts the attempt. `pick_duration_timesteps` controls how long the robot remains busy before it can start another attempt.

### Robots

Robots are configured by ID:

```json
"robots": {
  "R1": {
    "reachable_bins": [4, 5, 6],
    "pick_duration_timesteps": 1,
    "max_picks_per_timestep": 1
  },
  "R2": {
    "reachable_bins": [11],
    "pick_duration_timesteps": 1,
    "max_picks_per_timestep": 1
  }
}
```

The simulator does not assume exactly two robots. If the config contains `R1`, `R2`, and `R3`, the policy CSV must contain action columns for all three.

### Pick probabilities

Each robot needs a success probability for every item type:

```json
"pick_probabilities": {
  "R1": {"A": 0.8, "B": 0.6, "C": 0.7, "D": 0.3, "E": 0.5},
  "R2": {"A": 0.35, "B": 0.5, "C": 0.55, "D": 0.85, "E": 0.75}
}
```

All successful picks count as `+1`. There are no item-specific values.

## Policy CSV format

The policy CSV format depends on the agreed config. For 15 bins and 2 robots, the required columns are:

```text
state_id,
bin_1,bin_2,...,bin_15,
R1_pick_bin,R1_pick_item,
R2_pick_bin,R2_pick_item,
expected_remaining_successes
```

`expected_remaining_successes` is allowed but ignored by the simulator. It can be useful for algorithm debugging, but the simulator does not use it to make decisions.

For 20 bins and 3 robots, the expected format becomes:

```text
state_id,
bin_1,bin_2,...,bin_20,
R1_pick_bin,R1_pick_item,
R2_pick_bin,R2_pick_item,
R3_pick_bin,R3_pick_item,
expected_remaining_successes
```

The simulator validates the CSV against the config.

## Bin encoding

Bin cells are strings:

```text
EMPTY  no items
A      one A item
AB     one A and one B
AAD    two A items and one D
```

Item order inside a bin does not matter. The simulator canonicalizes bin cells, so `BA` and `AB` are treated as the same state.

## No-action encoding

A robot that should not pick uses:

```text
NONE,NONE
```

Example:

```text
R1_pick_bin = NONE
R1_pick_item = NONE
```

## Missing states

A missing state happens when the simulator reaches a conveyor state that is not present in the policy CSV.

Default behavior:

```json
"missing_state_behavior": "stop"
```

This records the missing state and stops that trial. This is recommended for debugging policy table coverage.

Alternative behavior:

```json
"missing_state_behavior": "no_action"
```

This records the missing state and continues with no robot actions for that timestep. This is not optimization; it is only a passive fallback.

## Invalid actions

An invalid action happens when the CSV recommends an illegal action, such as:

- picking from an unreachable bin
- picking an item that is not present
- using a malformed bin number
- using an unknown item type
- giving only one half of an action, such as `R1_pick_bin = 4` and `R1_pick_item = NONE`

Default behavior:

```json
"invalid_action_behavior": "stop"
```

Alternative behavior:

```json
"invalid_action_behavior": "skip"
```

This records the invalid action and skips that robot action. It does not choose a replacement action.

## Output CSV files

The simulator writes four CSV files.

### `run_results.csv`

One row per algorithm per trial.

Important columns:

```text
algorithm_name
policy_csv_file
trial_id
random_seed
num_bins
num_robots
elapsed_timesteps
total_successful_picks
total_failed_attempts
total_attempts
total_missed_items
successful_picks_per_1000_timesteps
missing_state_count
invalid_action_count
simulation_status
```

There are also per-robot and per-item columns, such as:

```text
R1_successful_picks
R1_failed_attempts
A_successful_picks
A_missed_items
```

### `summary_results.csv`

Aggregated results by algorithm.

Important columns:

```text
algorithm_name
num_trials
completed_trials
avg_elapsed_timesteps
avg_successful_picks
avg_failed_attempts
avg_total_attempts
avg_missed_items
avg_successful_picks_per_1000_timesteps
missing_state_count
invalid_action_count
```

### `missing_states.csv`

Every missing policy state encountered during simulation.

Important columns:

```text
algorithm_name
policy_csv_file
trial_id
timestep
state_key
bin_1 ... bin_N
reason
```

This file is used to debug policy CSV coverage or state encoding mismatches.

### `invalid_actions.csv`

Every invalid policy action encountered during simulation.

Important columns:

```text
algorithm_name
policy_csv_file
trial_id
timestep
state_key
robot_id
recommended_pick_bin
recommended_pick_item
bin_1 ... bin_N
reason
```

## Main metrics

For finite item tests, the main metric is usually:

```text
average successful picks per trial
```

For continuous-feed tests, the main metric is:

```text
successful_picks_per_1000_timesteps
```

Formula:

```text
successful_picks_per_1000_timesteps = successful_picks / elapsed_timesteps * 1000
```

## Continuous-feed mode

To use continuous mode, change:

```json
"mode": "continuous"
```

and configure feed settings:

```json
"feed": {
  "arrival_probability_per_movement": 0.7,
  "items_per_arrival": 1,
  "max_items_per_arrival": 1,
  "item_type_distribution": {"A": 1, "B": 1, "C": 1, "D": 1, "E": 1}
}
```

In continuous mode, the simulator runs until `max_timesteps` is reached.

## Important design notes

- The simulator uses the policy CSV as the source of robot actions.
- The simulator does not optimize or improve those actions.
- The simulator can compare multiple algorithms because each algorithm only needs to output a compatible policy CSV.
- The config and policy CSV must agree on bin count, robot IDs, and item types.
- Missing states are exported so policy generation problems can be diagnosed.
- Invalid actions are exported so algorithm output problems can be diagnosed.

## Running 10,000 random finite episodes

The simulator supports random starting states using the `random_initial_state` config block. This is for finite episodes where random items are placed into the first few bins before each trial begins.

Example config fields:

```json
"trials": 10000,
"mode": "finite",
"initial_state": {},
"random_initial_state": {
  "enabled": true,
  "bins": [1, 2, 3],
  "min_total_items": 0,
  "max_total_items": 5,
  "item_type_distribution": {"A": 1, "B": 1, "C": 1, "D": 1, "E": 1}
}
```

Run it with:

```bash
python simulator.py --config config.random_10000.example.json --policy dp=dp_policy.csv --output-dir outputs_random_10000
```

On macOS/Linux, use `python3` if needed:

```bash
python3 simulator.py --config config.random_10000.example.json --policy dp=dp_policy.csv --output-dir outputs_random_10000
```

Each trial gets a deterministic seed based on `random_seed + trial_id - 1`, so the same config and policy produce repeatable results.

Important: the policy CSV must contain rows for the states the random episodes can reach. If the policy table is incomplete, the simulator records those states in `missing_states.csv` and stops those trials when `missing_state_behavior` is `stop`.
