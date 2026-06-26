"""Interactive PBSTO demo in the PyBullet GUI.

Spawns a two-wheel robot and a randomized obstacle field, then runs
PBSTO online re-planning until the robot reaches the goal. The successful
trajectory is traced in red.

Usage:
    python robotSim.py
"""

import time

import numpy as np
import pybullet as p

from env import setup_world, random_obstacle_field, load_scene
from sim import run_trial


def main(seed=42):
    p.connect(p.GUI)
    setup_world()

    obstacles = random_obstacle_field(seed=seed)
    robot, target, _ = load_scene([0, 0, 0.2], [6, 0, 0], obstacles)

    success = run_trial(robot, target, planner="pbsto", seed=seed)
    print("SUCCESS" if success else "FAIL")

    # Keep the GUI open for a few seconds so the trajectory can be inspected.
    time.sleep(5)
    p.disconnect()


if __name__ == "__main__":
    main()
