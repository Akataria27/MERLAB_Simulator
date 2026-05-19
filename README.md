# Recycling Simulator

## Command-line usage

Run all policies listed in the config for the config's default number of trials:

```bash
python simulator.py --config config_example.json --run-title first_test
```

Run 1000 episodes per policy:

```bash
python simulator.py --config config_example.json --run-title four_policy_test --trials 1000
```

Set the base random seed:

```bash
python simulator.py --config config_example.json --run-title seeded_test --trials 1000 --seed 42
```

Every run creates a new timestamped output folder:

```text
outputs/<run_title>_<YYYY-MM-DD_HH-MM-SS>/
  summary_results.csv
  missing_states.csv
```

If `outputs.write_run_results` is `true`, the run folder also includes:

```text
run_results.csv
```

Existing output files are not overwritten because every run creates a new folder.

## Main purpose

The simulator tests policies against the same conveyor setup. It does not optimize, repair, train, or improve policies. CSV-based algorithms produce policy CSVs. The simulator loads those CSVs and executes the recommended actions.

## Adding a new algorithm

To test a new CSV-based algorithm:

1. Put the CSV file somewhere accessible, commonly in a `policies/` folder.
2. Add it to the `policies` list in the config:

```json
{
  "name": "new_algorithm",
  "type": "csv",
  "file": "policies/new_algorithm.csv"
}
```

Then run:

```bash
python simulator.py --config config_example.json --run-title new_algorithm_test --trials 1000
```

No simulator code changes are needed as long as the CSV follows the expected format.

## Built-in policies

The simulator has two built-in policies that do not require CSV files.

### Random

```json
{
  "name": "random",
  "type": "random"
}
```

For each available robot, if there are items in reachable bins, the robot must attempt a valid random pick. It only does nothing when no reachable items exist.

### Highest probability

```json
{
  "name": "highest_probability",
  "type": "highest_probability"
}
```

For each available robot, this policy checks all reachable items and picks one with the highest success probability for that robot. Ties are broken randomly.

## Config structure

Important fields:

```json
{
  "mode": "finite",
  "num_bins": 15,
  "trials": 1000,
  "random_seed": 1,
  "bin_move_timesteps": 1,
  "default_pick_duration_timesteps": 1,
  "robots": [],
  "pick_probabilities": {},
  "random_initial_state": {},
  "policies": [],
  "outputs": {}
}
```

Only finite mode is implemented. Continuous mode is intentionally not functional yet.

## Random finite-mode initial state

This config means each trial starts with 0 to 5 total items randomly placed across bins 1, 2, and 3:

```json
"random_initial_state": {
  "enabled": true,
  "bins": [1, 2, 3],
  "min_total_items": 0,
  "max_total_items": 5,
  "item_type_distribution": {
    "A": 1,
    "B": 1,
    "C": 1,
    "D": 1,
    "E": 1
  }
}
```

The item range is total items across the listed bins, not per bin.

## Policy CSV format

The simulator uses `num_bins` and the `robots` list from the config to decide which columns to read.

For 15 bins and robots `R1`, `R2`, the required columns are:

```text
state_id,
bin_1,
bin_2,
...
bin_15,
R1_pick_bin,
R1_pick_item,
R2_pick_bin,
R2_pick_item
```

Extra columns after these are allowed and ignored, such as:

```text
expected_remaining_successes
```

For 20 bins and 3 robots, the same pattern applies:

```text
state_id,
bin_1,
...
bin_20,
R1_pick_bin,
R1_pick_item,
R2_pick_bin,
R2_pick_item,
R3_pick_bin,
R3_pick_item
```

The first column should be `state_id`, but state matching is done using the bin columns, not the state ID.

## Bin values

Empty bins should be written as:

```text
EMPTY
```

No action should be written as:

```text
NONE
```

Bin contents are canonicalized alphabetically by the simulator:

```text
BA -> AB
ADA -> AAD
```

## Missing states

If a CSV policy reaches a conveyor state that is not in the CSV, the simulator records it in `missing_states.csv`, performs no picks for that timestep, moves the belt forward as usual, and continues the trial.

Trials with at least one missing state are marked as:

```text
completed_with_missing_states
```

## Outputs

### summary_results.csv

One row per policy with aggregate performance:

```text
algorithm_name
policy_type
policy_csv_file
num_trials
completed_trials
completed_with_missing_state_trials
avg_successful_picks
avg_failed_attempts
avg_missed_items
avg_total_attempts
avg_elapsed_timesteps
avg_successful_picks_per_1000_timesteps
total_missing_states
```

### missing_states.csv

One row per missing state encountered:

```text
algorithm_name
policy_csv_file
trial_id
timestep
state_key
bin_1 ... bin_N
robot_busy_status
reason
```

### run_results.csv

Only written when enabled:

```json
"outputs": {
  "write_run_results": true
}
```

It contains one row per policy per trial.
