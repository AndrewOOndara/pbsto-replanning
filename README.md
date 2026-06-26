# Real-Time Motion Re-Planning Under Clutter and Uncertainty

A MuJoCo implementation of physics-based stochastic trajectory optimization (PBSTO) for online motion re-planning on a differential-drive mobile robot. Based on [Agboh & Dogar, 2018](https://arxiv.org/abs/1807.09049), this repo adapts the original arm-grasping algorithm to a two-wheel robot and benchmarks online re-planning against a single-shot baseline in a randomized cluttered environment.

## Result

Across 50 randomized cluttered runs on a two-wheel differential-drive robot in MuJoCo:

| Method | Path Completion Rate |
|--------|---------------------|
| **Online re-planning (PBSTO)** | **56%** |
| Naive (single offline plan) | 14% |

PBSTO with candidate selection quadruples the success rate compared to a single-shot naive plan under stochastic disturbances and unmodeled clutter, validating the core claim of the original paper on a different platform.

## How It Works

At each replanning step, PBSTO samples ``num_samples`` candidate control sequences by perturbing a nominal forward trajectory with Gaussian noise. Each candidate is rolled out in MuJoCo from the current state (with state save/restore so the prediction doesn't affect the real robot) and scored by the minimum distance to the goal achieved during the rollout. The lowest-cost candidate is then executed for real. If the robot doesn't reach the goal, the state is reset and a fresh batch of candidates is sampled.

## Repo Layout

```
.
├── env.py              # Randomized obstacle field + MJCF scene builder
├── planner.py          # PBSTO sampler + naive baseline plan
├── sim.py              # Single-trial simulation runner
├── benchmark.py        # 50-trial PBSTO vs naive comparison
├── robotSim.py         # Interactive MuJoCo viewer demo
└── requirements.txt
```

## Setup

Tested with Python 3.10, 3.11, and 3.12. MuJoCo ships prebuilt wheels for macOS, Linux, and Windows, so no compilation is required.

```bash
git clone https://github.com/AndrewOOndara/pbsto-replanning.git
cd pbsto-replanning
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run the Demo

```bash
python robotSim.py
```

Opens the MuJoCo viewer showing PBSTO navigating one randomized scene.

## Reproduce the Benchmark

```bash
python benchmark.py --planner pbsto --trials 50
python benchmark.py --planner naive --trials 50
```

Runs headless. Each trial uses a fresh randomized obstacle layout. Reproducible via the ``--seed`` flag.

## Key Parameters

Defined in `sim.py`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `horizon` | Rollout length per plan | `12` |
| `num_samples` | Candidate plans sampled per replan | `15` |
| `std` | Gaussian noise stddev per control element | `0.2` |
| `max_replans` | Replanning iterations per trial | `10` |
| `threshold` | Goal radius (m) | `0.60` |
| `n_steps_pred` | Physics sub-steps per control during prediction | `100` |
| `n_steps` (execution) | Physics sub-steps per control during execution | `200` |

## Citation

If this implementation helps your work, please cite the original paper:

```bibtex
@inproceedings{agboh2018real,
  title={Real-Time Online Re-Planning for Grasping Under Clutter and Uncertainty},
  author={Agboh, Wisdom C. and Dogar, Mehmet R.},
  booktitle={IEEE-RAS International Conference on Humanoid Robotics (Humanoids)},
  year={2018},
  url={https://arxiv.org/abs/1807.09049}
}
```

## Notes

Conducted as research with the [Robotics and Physical Interactions Lab](https://robotpilab.github.io/) at Rice University.

## License

MIT
