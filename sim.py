"""Run a single navigation trial under one of the planners.

Wraps the MuJoCo stepping loop so the benchmark harness can call ``run_trial``
once per random seed and just consume the success/failure result.
"""

import numpy as np
import mujoco

from planner import naive_plan, pbsto_step


def get_xy(model, data, name):
    """Return the planar position of a body as ``[x, y]``.

    Args:
        model: ``mujoco.MjModel``.
        data: ``mujoco.MjData``.
        name: Body name as declared in the MJCF.
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    return data.xpos[body_id, :2].copy()


def step_robot(model, data, forward, turn, speed=10, n_steps=10, on_step=None):
    """Apply differential-drive velocities and step physics forward.

    Differential-drive mixing: ``v_left = forward - turn``,
    ``v_right = forward + turn``. The two wheel actuators correspond to the
    first and second entries of ``data.ctrl``.

    Args:
        model: ``mujoco.MjModel``.
        data: ``mujoco.MjData``.
        forward: Forward velocity command.
        turn: Yaw rate command.
        speed: Wheel velocity scale factor.
        n_steps: Number of physics sub-steps per control interval.
        on_step: Optional zero-arg callable invoked after each physics step.
            Used by the GUI demo to sync the viewer.
    """
    data.ctrl[0] = (forward - turn) * speed   # left wheel
    data.ctrl[1] = (forward + turn) * speed   # right wheel
    for _ in range(n_steps):
        mujoco.mj_step(model, data)
        if on_step is not None:
            on_step()


def run_trial(model, data, planner="pbsto", horizon=10, max_replans=20,
              std=0.1, threshold=0.6, seed=None, on_step=None):
    """Drive the robot toward the target and return whether it arrived.

    For ``planner="pbsto"``, a fresh stochastic plan is sampled before each
    rollout, up to ``max_replans`` times. For ``planner="naive"``, the nominal
    forward plan is executed once with no replanning.

    Args:
        model: ``mujoco.MjModel``.
        data: ``mujoco.MjData``.
        planner: ``"pbsto"`` or ``"naive"``.
        horizon: Length of each control rollout in steps.
        max_replans: Maximum number of replanning iterations for PBSTO.
        std: PBSTO sampling noise standard deviation.
        threshold: Distance (m) to the goal that counts as success.
        seed: RNG seed for reproducible noise.
        on_step: Optional zero-arg callable invoked after each physics step.

    Returns:
        bool: ``True`` if the robot ended within ``threshold`` of the goal.
    """
    rng = np.random.default_rng(seed)
    nominal = naive_plan(horizon)

    iterations = max_replans if planner == "pbsto" else 1
    for _ in range(iterations):
        controls = pbsto_step(nominal, std, rng) if planner == "pbsto" else nominal
        for forward, turn in controls:
            step_robot(model, data, forward, turn, on_step=on_step)
            dist = np.linalg.norm(get_xy(model, data, "turtle")
                                  - get_xy(model, data, "target"))
            if dist < threshold:
                return True
    return False
