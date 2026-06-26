"""Interactive PBSTO demo in the MuJoCo viewer.

Builds a randomized obstacle field and runs PBSTO online re-planning until the
robot reaches the goal, with the MuJoCo viewer rendering each physics step and
drawing a red trail of the robot's path.

Usage:
    mjpython robotSim.py
"""

import time

import numpy as np
import mujoco
import mujoco.viewer

from env import random_obstacle_field, build_scene
from sim import run_trial


# Drop a trail marker every N physics steps to avoid clutter.
TRAIL_EVERY = 10
TRAIL_RGBA = [1.0, 0.2, 0.2, 0.7]


def add_trail_sphere(scene, pos, size=0.04):
    """Append a tiny sphere geom to the viewer's user scene at ``pos``."""
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=np.array([size, 0, 0]),
        pos=np.asarray(pos, dtype=np.float64),
        mat=np.eye(3).flatten(),
        rgba=np.array(TRAIL_RGBA),
    )
    scene.ngeom += 1


def main(seed=42):
    obstacles = random_obstacle_field(seed=seed)
    model, data = build_scene(obstacles)

    turtle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "turtle")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        dt = model.opt.timestep
        step_counter = [0]

        def real_time_sync():
            step_counter[0] += 1
            if step_counter[0] % TRAIL_EVERY == 0:
                pos = data.xpos[turtle_id].copy()
                pos[2] = 0.02  # drop trail just above floor
                add_trail_sphere(viewer.user_scn, pos)
            viewer.sync()
            time.sleep(dt)

        success = run_trial(
            model, data, planner="pbsto", seed=seed,
            on_step=real_time_sync,
        )
        print("SUCCESS" if success else "FAIL")

        # Hold the viewer open for a few seconds so the result can be inspected.
        end = time.time() + 5
        while viewer.is_running() and time.time() < end:
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
