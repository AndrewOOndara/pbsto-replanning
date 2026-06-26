# Real-Time Motion Re-Planning Under Clutter and Uncertainty

  A PyBullet implementation and benchmark of physics-based stochastic trajectory optimization (PBSTO) for online motion re-planning in cluttered, uncertain
  environments. This repository reimplements the algorithm proposed in [Agboh & Dogar, 2018](https://arxiv.org/abs/1807.09049) and benchmarks online re-planning
  against a naive baseline across 50 randomized obstacle configurations.

  ## Result

  Across 50 randomized cluttered runs on a two-wheel URDF robot:

  | Method | Path Completion Rate |
  |--------|---------------------|
  | **Online re-planning (PBSTO)** | **78%** |
  | Naive (single offline plan) | 43% |

  Online re-planning nearly doubles success rate under stochastic disturbances and unmodeled clutter, validating the core claim of the paper in a different
  platform than the original (two-wheel mobile robot instead of fixed-base arm).

  ## Demo

  *(GIF or video of robot navigating cluttered scene — add here)*

  ## What's Implemented

  - PyBullet physics simulation with two-wheel URDF robot
  - Stochastic forward dynamics rollouts for trajectory cost evaluation
  - Online re-planning loop triggered on state divergence from nominal trajectory
  - Randomized environment generator (obstacle count, position, friction)
  - Benchmarking harness over N trials with success/failure logging
  - Naive baseline planner for direct comparison

  ## Repo Layout

  .
  ├── envs/                 # PyBullet environments + URDF assets
  ├── planner/              # PBSTO + naive planner implementations
  ├── benchmark/            # Multi-run evaluation harness
  ├── results/              # Logged trajectories, metrics, plots
  └── main.py               # Entry point

  ## Setup

  ```bash
  git clone https://github.com/AndrewOOndara/Real-Time-Online-Re-Planning-for-Grasping-Under-Clutter-and-Uncertainty.git
  cd Real-Time-Online-Re-Planning-for-Grasping-Under-Clutter-and-Uncertainty
  pip install -r requirements.txt

  Reproduce the Benchmark

  Single run with GUI:
  python main.py --mode gui --planner pbsto

  Full 50-trial benchmark (headless):
  python benchmark/run.py --trials 50 --planner pbsto
  python benchmark/run.py --trials 50 --planner naive
  python benchmark/plot.py

  Citation

  If this implementation helps your work, please cite the original paper:

  @inproceedings{agboh2018real,
    title={Real-Time Online Re-Planning for Grasping Under Clutter and Uncertainty},
    author={Agboh, Wisdom C. and Dogar, Mehmet R.},
    booktitle={IEEE-RAS International Conference on Humanoid Robotics (Humanoids)},
    year={2018},
    url={https://arxiv.org/abs/1807.09049}
  }

  Notes

  Conducted as research with the Robotics and Physical Interactions Lab at Rice University.

  License

  MIT
