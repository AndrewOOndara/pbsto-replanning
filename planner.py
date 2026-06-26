"""Planners under comparison: PBSTO and a naive open-loop baseline.

The two share a control representation: a length-``horizon`` array of
``(forward, turn)`` pairs that a differential-drive robot can execute one
step at a time.
"""

import numpy as np


def naive_plan(horizon, control_dim=2):
    """Return a fixed open-loop plan that drives the robot straight forward.

    Used as the no-replanning baseline: the robot commits to driving forward
    for the full horizon with zero turn, regardless of what the environment
    looks like.

    Args:
        horizon: Number of control steps.
        control_dim: Width of each control vector. Defaults to 2
            (``[forward, turn]``).

    Returns:
        np.ndarray: ``(horizon, control_dim)`` plan with column 0 = 1.0 and
        all other columns = 0.0.
    """
    plan = np.zeros((horizon, control_dim))
    plan[:, 0] = 1.0
    return plan


def pbsto_step(nominal, std, rng=None):
    """Sample one PBSTO candidate by perturbing the nominal plan with noise.

    Implements the core sampling step of physics-based stochastic trajectory
    optimization. Each control vector in the nominal sequence is perturbed by
    isotropic Gaussian noise with standard deviation ``std``. The caller
    executes the sampled sequence in simulation and decides whether to keep
    it.

    Args:
        nominal: ``(horizon, control_dim)`` nominal plan to perturb.
        std: Standard deviation of the Gaussian noise applied per element.
        rng: Optional ``np.random.Generator`` for reproducibility.

    Returns:
        np.ndarray: Perturbed plan with the same shape as ``nominal``.
    """
    rng = rng or np.random.default_rng()
    noise = rng.normal(0.0, std, size=nominal.shape)
    return nominal + noise
