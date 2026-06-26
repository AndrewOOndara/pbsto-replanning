# Real-Time Motion Re-Planning Under Clutter and Uncertainty

A PyBullet implementation of physics-based stochastic trajectory optimization (PBSTO) for online motion re-planning on a differential-drive mobile robot. Based on [Agboh & Dogar, 2018](https://arxiv.org/abs/1807.09049), this repo adapts the original arm-grasping algorithm to a two-wheel URDF robot and benchmarks online re-planning against a single-shot baseline in a randomized cluttered environment.

## Result

Across 50 randomized cluttered runs on a two-wheel URDF robot:

| Method | Path Completion Rate |
|--------|---------------------|
| **Online re-planning (PBSTO)** | **78%** |
| Naive (single offline plan) | 43% |

Online re-planning nearly doubles success rate under stochastic disturbances and unmodeled clutter, validating the core claim of the original paper on a different platform.

## How It Works

At each replanning step, PBSTO samples candidate control sequences by perturbing a nominal forward trajectory with Gaussian noise. Each candidate is rolled out in PyBullet to estimate goal progress. The best candidate is executed for one step, then the loop repeats from the new state, so stochastic dynamics and unmodeled obstacles can be corrected for online rather than committed to up front.

## Repo Layout

```
.
├── env.py              # Randomized obstacle field + scene loader
├── planner.py          # PBSTO sampler + naive baseline plan
├── sim.py              # Single-trial simulation runner
├── benchmark.py        # 50-trial PBSTO vs naive comparison
├── robotSim.py         # Interactive GUI demo
├── urdf/               # Robot, target, and obstacle URDFs
└── requirements.txt
```

## Setup

Tested with Python 3.10 and 3.11. PyBullet ships Linux-only wheels, so macOS users will compile from source (Xcode command line tools required).

```bash
# macOS only, if you don't already have build tools:
xcode-select --install

git clone https://github.com/AndrewOOndara/pbsto-replanning.git
cd pbsto-replanning
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run the Demo

```bash
python robotSim.py
```

Opens a PyBullet GUI showing PBSTO navigating one randomized scene.

## Reproduce the Benchmark

```bash
python benchmark.py --planner pbsto --trials 50
python benchmark.py --planner naive --trials 50
```

Runs headless. Each trial uses a fresh randomized obstacle layout. Reproducible via the ``--seed`` flag.

## Key Parameters

Defined in `planner.py` and `sim.py`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `horizon` | Rollout length per plan | `10` |
| `std` | PBSTO sampling noise stddev | `0.1` |
| `max_replans` | Replanning iterations per trial | `20` |
| `threshold` | Goal radius (m) | `0.60` |
| `speed` | Wheel velocity scale | `10` |

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
