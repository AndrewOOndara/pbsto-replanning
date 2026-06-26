"""Randomized cluttered environment for the navigation benchmark.

Provides helpers to seed PyBullet with a flat ground plane, a two-wheel turtle
robot, a goal marker, and a randomized field of cuboid obstacles in front of
the robot's spawn.
"""

import numpy as np
import pybullet as p
import pybullet_data as pd


def setup_world():
    """Configure PyBullet defaults and load the ground plane.

    Returns:
        int: PyBullet body ID for the loaded ground plane.
    """
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setRealTimeSimulation(0)
    p.setGravity(0, 0, -10)
    return p.loadURDF("plane100.urdf")


def random_obstacle_field(n=16, x_range=(1.0, 6.0), y_range=(-1.5, 1.5), seed=None):
    """Sample ``n`` obstacle positions uniformly in a rectangle in front of the robot.

    Args:
        n: Number of obstacles to sample.
        x_range: ``(min, max)`` x bounds.
        y_range: ``(min, max)`` y bounds.
        seed: RNG seed. Pass an int for reproducible layouts.

    Returns:
        list[list[float]]: List of ``[x, y, z]`` obstacle positions.
    """
    rng = np.random.default_rng(seed)
    xs = rng.uniform(*x_range, n)
    ys = rng.uniform(*y_range, n)
    return [[float(x), float(y), 1.0] for x, y in zip(xs, ys)]


def load_scene(robot_pos, target_pos, obstacle_positions, robot_yaw=0.0):
    """Spawn the robot, goal marker, and obstacles in the world.

    Args:
        robot_pos: 3-vector ``[x, y, z]`` spawn position for the robot.
        target_pos: 3-vector ``[x, y, z]`` goal position.
        obstacle_positions: Iterable of ``[x, y, z]`` obstacle positions.
        robot_yaw: Initial heading of the robot in radians.

    Returns:
        tuple: ``(robot_id, target_id, list_of_obstacle_ids)``.
    """
    ori = p.getQuaternionFromEuler([0, 0, robot_yaw])
    robot = p.loadURDF("urdf/most_simple_turtle.urdf", robot_pos, ori)
    target = p.loadURDF("urdf/target.urdf", target_pos)
    obstacles = [p.loadURDF("urdf/box.urdf", pos) for pos in obstacle_positions]
    return robot, target, obstacles
