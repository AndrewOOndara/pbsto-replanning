"""Interactive PBSTO demo in the MuJoCo viewer.

Builds a randomized obstacle field and runs PBSTO online re-planning until the
robot reaches the goal, with the MuJoCo viewer rendering each physics step.

Usage:
    python robotSim.py
"""

import time

import mujoco
import mujoco.viewer

from env import random_obstacle_field, build_scene
from sim import run_trial


def main(seed=42):
    obstacles = random_obstacle_field(seed=seed)
    model, data = build_scene(obstacles)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        success = run_trial(
            model, data, planner="pbsto", seed=seed,
            on_step=viewer.sync,
        )
        print("SUCCESS" if success else "FAIL")

        # Hold the viewer open for a few seconds so the result can be inspected.
        end = time.time() + 5
        while viewer.is_running() and time.time() < end:
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
