"""Benchmark PBSTO vs the naive baseline across N randomized trials.

Each trial spawns a fresh randomized obstacle field, runs the chosen planner
once, and records success. Prints the final success rate.

Usage:
    python benchmark.py --planner pbsto --trials 50
    python benchmark.py --planner naive --trials 50
"""

import argparse

import pybullet as p

from env import setup_world, random_obstacle_field, load_scene
from sim import run_trial


def benchmark(planner, n_trials=50, seed=42):
    """Run ``n_trials`` randomized trials of ``planner`` and report success rate.

    Args:
        planner: ``"pbsto"`` or ``"naive"``.
        n_trials: Number of randomized trials.
        seed: Base RNG seed. Trial ``i`` uses ``seed + i`` for its obstacle
            layout and planner noise, so runs are reproducible.

    Returns:
        float: Fraction of trials in which the robot reached the goal.
    """
    successes = 0
    for trial in range(n_trials):
        p.connect(p.DIRECT)
        setup_world()
        obstacles = random_obstacle_field(seed=seed + trial)
        robot, target, _ = load_scene([0, 0, 0.2], [6, 0, 0], obstacles)
        result = run_trial(robot, target, planner=planner, seed=seed + trial)
        successes += int(result)
        p.disconnect()
        print(f"  Trial {trial + 1:>3}/{n_trials}: {'SUCCESS' if result else 'FAIL'}")
    rate = successes / n_trials
    print(f"\n{planner.upper()}: {successes}/{n_trials} = {rate:.1%}")
    return rate


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--planner", choices=["pbsto", "naive"], required=True,
                        help="Which planner to benchmark.")
    parser.add_argument("--trials", type=int, default=50,
                        help="Number of randomized trials. Default: 50.")
    parser.add_argument("--seed", type=int, default=42,
                        help="Base RNG seed for reproducibility. Default: 42.")
    args = parser.parse_args()
    benchmark(args.planner, args.trials, args.seed)
