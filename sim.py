"""Run a single navigation trial under one of the planners.

Wraps the PyBullet stepping loop so the benchmark harness can call
``run_trial`` once per random seed and just consume the success/failure
result.
"""

import numpy as np
import pybullet as p

from planner import naive_plan, pbsto_step


def get_xy(body):
    """Return the planar position of a PyBullet body as ``[x, y]``."""
    pos, _ = p.getBasePositionAndOrientation(body)
    return np.array([pos[0], pos[1]])


def step_robot(robot, forward, turn, speed=10, n_steps=20):
    """Apply differential-drive velocities and step physics forward.

    The robot's left and right wheels are joints 0 and 1 respectively. We
    mix the (forward, turn) command into wheel velocities via the standard
    differential-drive form: ``v_left = forward - turn``, ``v_right =
    forward + turn``.

    Args:
        robot: PyBullet body ID for the robot.
        forward: Forward velocity command.
        turn: Yaw rate command.
        speed: Wheel velocity scale factor.
        n_steps: Number of physics sub-steps to integrate.
    """
    p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL,
                            targetVelocity=(forward - turn) * speed, force=1000)
    p.setJointMotorControl2(robot, 1, p.VELOCITY_CONTROL,
                            targetVelocity=(forward + turn) * speed, force=1000)
    for _ in range(n_steps):
        p.stepSimulation()


def run_trial(robot, target, planner="pbsto", horizon=10, max_replans=20,
              std=0.1, threshold=0.6, seed=None):
    """Drive the robot toward the target and return whether it arrived.

    For ``planner="pbsto"``, a fresh stochastic plan is sampled before each
    rollout, up to ``max_replans`` times. For ``planner="naive"``, the
    nominal forward plan is executed once with no replanning.

    Args:
        robot: PyBullet body ID for the robot.
        target: PyBullet body ID for the goal marker.
        planner: ``"pbsto"`` or ``"naive"``.
        horizon: Length of each control rollout in steps.
        max_replans: Maximum number of replanning iterations for PBSTO.
        std: PBSTO sampling noise standard deviation.
        threshold: Distance (m) to the goal that counts as success.
        seed: RNG seed for reproducible noise.

    Returns:
        bool: ``True`` if the robot ended within ``threshold`` of the goal.
    """
    rng = np.random.default_rng(seed)
    nominal = naive_plan(horizon)

    iterations = max_replans if planner == "pbsto" else 1
    for _ in range(iterations):
        controls = pbsto_step(nominal, std, rng) if planner == "pbsto" else nominal
        for forward, turn in controls:
            step_robot(robot, forward, turn)
            dist = np.linalg.norm(get_xy(robot) - get_xy(target))
            if dist < threshold:
                return True
    return False
