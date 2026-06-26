"""Benchmark PBSTO vs the naive baseline across N randomized trials.

Each trial builds a fresh MuJoCo scene with a randomized obstacle field, runs
the chosen planner once, and records success. Prints the final success rate.

Usage:
    python benchmark.py --planner pbsto --trials 50              # headless, max speed
    mjpython benchmark.py --planner pbsto --viewer --speed 5     # watch at 5x
"""

import argparse
import time

from env import random_obstacle_field, build_scene
from sim import run_trial


def benchmark(planner, n_trials=50, seed=42, viewer=False, speed=1.0):
    """Run ``n_trials`` randomized trials of ``planner`` and report success rate.

    Args:
        planner: ``"pbsto"`` or ``"naive"``.
        n_trials: Number of randomized trials.
        seed: Base RNG seed. Trial ``i`` uses ``seed + i`` for its obstacle
            layout and planner noise, so runs are reproducible.
        viewer: If True, open the MuJoCo viewer for each trial (requires
            ``mjpython`` on macOS).
        speed: Wall-clock playback multiplier when ``viewer`` is on. ``1.0`` is
            real-time, ``5.0`` is 5x faster, ``0`` is uncapped.

    Returns:
        float: Fraction of trials in which the robot reached the goal.
    """
    successes = 0
    for trial in range(n_trials):
        obstacles = random_obstacle_field(seed=seed + trial)
        model, data = build_scene(obstacles)

        on_step = None
        viewer_ctx = None
        if viewer:
            import mujoco.viewer
            viewer_ctx = mujoco.viewer.launch_passive(model, data)
            dt = model.opt.timestep
            sleep_time = dt / speed if speed > 0 else 0

            def on_step():
                viewer_ctx.sync()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        try:
            result = run_trial(model, data, planner=planner,
                               seed=seed + trial, on_step=on_step)
        finally:
            if viewer_ctx is not None:
                viewer_ctx.close()

        successes += int(result)
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
    parser.add_argument("--viewer", action="store_true",
                        help="Open the MuJoCo viewer (use mjpython on macOS).")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier when --viewer is on. "
                             "1.0 = real-time, 5.0 = 5x, 0 = uncapped.")
    args = parser.parse_args()
    benchmark(args.planner, args.trials, args.seed, args.viewer, args.speed)
