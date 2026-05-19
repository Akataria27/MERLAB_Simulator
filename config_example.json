{
  "mode": "finite",
  "num_bins": 15,
  "trials": 1000,
  "random_seed": 1,
  "bin_move_timesteps": 1,
  "default_pick_duration_timesteps": 1,
  "item_types": [
    "A",
    "B",
    "C",
    "D",
    "E"
  ],
  "robots": [
    {
      "name": "R1",
      "reachable_bins": [
        4,
        5,
        6
      ],
      "pick_duration_timesteps": 1
    },
    {
      "name": "R2",
      "reachable_bins": [
        11
      ],
      "pick_duration_timesteps": 1
    }
  ],
  "pick_probabilities": {
    "R1": {
      "A": 0.7,
      "B": 0.5,
      "C": 0.8,
      "D": 0.6,
      "E": 0.4
    },
    "R2": {
      "A": 0.3,
      "B": 0.85,
      "C": 0.45,
      "D": 0.75,
      "E": 0.65
    }
  },
  "random_initial_state": {
    "enabled": true,
    "bins": [
      1,
      2,
      3
    ],
    "min_total_items": 0,
    "max_total_items": 5,
    "item_type_distribution": {
      "A": 1,
      "B": 1,
      "C": 1,
      "D": 1,
      "E": 1
    }
  },
  "policies": [
    {
      "name": "random",
      "type": "random"
    },
    {
      "name": "highest_probability",
      "type": "highest_probability"
    }
  ],
  "outputs": {
    "base_dir": "outputs",
    "write_run_results": false
  }
}
