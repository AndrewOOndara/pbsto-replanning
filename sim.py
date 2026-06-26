"""Run a single navigation trial under one of the planners.

Implements the full PBSTO control loop with model-predictive candidate
selection: at each replan, sample ``num_samples`` noisy control sequences,
predict each by rolling forward simulation from the current state, and
execute the candidate with the lowest predicted goal distance.
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


def reset_robot(model, data):
    """Reset the robot's qpos and qvel to the model's keyframe-free defaults."""
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)


def save_state(data):
    """Snapshot ``data`` enough to restore via :func:`restore_state`."""
    return (data.qpos.copy(), data.qvel.copy(), float(data.time))


def restore_state(model, data, snapshot):
    """Restore a snapshot produced by :func:`save_state`."""
    data.qpos[:] = snapshot[0]
    data.qvel[:] = snapshot[1]
    data.time = snapshot[2]
    mujoco.mj_forward(model, data)


def step_robot(model, data, forward, turn, speed=8, n_steps=200, on_step=None):
    """Apply differential-drive velocities and step physics forward.

    Differential-drive mixing: ``v_left = forward - turn``,
    ``v_right = forward + turn``.

    Args:
        model: ``mujoco.MjModel``.
        data: ``mujoco.MjData``.
        forward: Forward velocity command.
        turn: Yaw rate command.
        speed: Wheel velocity scale factor.
        n_steps: Number of physics sub-steps per control interval.
        on_step: Optional zero-arg callable invoked after each physics step.
    """
    data.ctrl[0] = (forward - turn) * speed
    data.ctrl[1] = (forward + turn) * speed
    for _ in range(n_steps):
        mujoco.mj_step(model, data)
        if on_step is not None:
            on_step()


def rollout_cost(model, data, controls, target_xy, n_steps_pred):
    """Roll out ``controls`` in-place from the current state and return cost.

    Cost is the minimum distance to the goal achieved during the rollout. Lower
    is better. Caller is responsible for snapshotting/restoring state around
    this call.

    Args:
        model: ``mujoco.MjModel``.
        data: ``mujoco.MjData``.
        controls: ``(horizon, 2)`` array of ``(forward, turn)`` commands.
        target_xy: 2-vector goal position.
        n_steps_pred: Physics sub-steps per control during prediction. Smaller
            than execution to keep rollouts cheap.

    Returns:
        float: Best (smallest) goal distance reached over the rollout.
    """
    best_dist = float("inf")
    for forward, turn in controls:
        step_robot(model, data, forward, turn, n_steps=n_steps_pred)
        dist = float(np.linalg.norm(get_xy(model, data, "turtle") - target_xy))
        if dist < best_dist:
            best_dist = dist
    return best_dist


def select_best_plan(model, data, nominal, num_samples, std, rng,
                     n_steps_pred=100):
    """Sample ``num_samples`` noisy plans, score each, return the lowest-cost one.

    Performs forward simulation for each candidate then restores the original
    state, so the caller can execute the selected plan from where it started.

    Args:
        model: ``mujoco.MjModel``.
        data: ``mujoco.MjData``.
        nominal: ``(horizon, 2)`` nominal control sequence.
        num_samples: Number of candidate trajectories to sample.
        std: Gaussian noise standard deviation per control element.
        rng: ``np.random.Generator``.
        n_steps_pred: Physics sub-steps per control during prediction.

    Returns:
        np.ndarray: The lowest-cost candidate control sequence.
    """
    snapshot = save_state(data)
    target_xy = get_xy(model, data, "target")

    best_cost = float("inf")
    best_controls = nominal.copy()

    for _ in range(num_samples):
        restore_state(model, data, snapshot)
        candidate = pbsto_step(nominal, std, rng)
        cost = rollout_cost(model, data, candidate, target_xy, n_steps_pred)
        if cost < best_cost:
            best_cost = cost
            best_controls = candidate

    restore_state(model, data, snapshot)
    return best_controls


def run_trial(model, data, planner="pbsto", horizon=12, max_replans=10,
              num_samples=15, std=0.2, threshold=0.6, seed=None, on_step=None):
    """Drive the robot toward the target and return whether it arrived.

    For ``planner="pbsto"``, runs model-predictive PBSTO: at each replan, sample
    ``num_samples`` noisy plans, select the one with lowest predicted goal
    distance, and execute it in the real simulation. For ``planner="naive"``,
    executes the nominal forward plan once with no replanning or selection.

    Args:
        model: ``mujoco.MjModel``.
        data: ``mujoco.MjData``.
        planner: ``"pbsto"`` or ``"naive"``.
        horizon: Length of each control rollout in steps.
        max_replans: Maximum number of replanning iterations for PBSTO.
        num_samples: Number of candidate trajectories sampled per replan.
        std: PBSTO sampling noise standard deviation.
        threshold: Distance (m) to the goal that counts as success.
        seed: RNG seed for reproducible noise.
        on_step: Optional zero-arg callable invoked after each physics step
            during *execution* (not during prediction rollouts).

    Returns:
        bool: ``True`` if the robot ended within ``threshold`` of the goal.
    """
    rng = np.random.default_rng(seed)
    nominal = naive_plan(horizon)

    if planner == "naive":
        for forward, turn in nominal:
            step_robot(model, data, forward, turn, on_step=on_step)
            dist = np.linalg.norm(get_xy(model, data, "turtle")
                                  - get_xy(model, data, "target"))
            if dist < threshold:
                return True
        return False

    # PBSTO: best-of-N with reset between attempts. Each attempt samples
    # `num_samples` noisy plans, scores each by predicted goal distance, and
    # executes the best from origin. If it doesn't reach the goal, reset and
    # try a fresh batch of candidates.
    for _ in range(max_replans):
        reset_robot(model, data)
        plan = select_best_plan(model, data, nominal, num_samples, std, rng)
        for forward, turn in plan:
            step_robot(model, data, forward, turn, on_step=on_step)
            dist = np.linalg.norm(get_xy(model, data, "turtle")
                                  - get_xy(model, data, "target"))
            if dist < threshold:
                return True
    return False
