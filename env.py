"""Randomized cluttered environment for the MuJoCo navigation benchmark.

Builds an inline MJCF scene with a flat ground plane, a two-wheel differential-
drive robot, a goal sphere, and a randomized field of cuboid obstacles in front
of the robot's spawn. Each call to :func:`build_scene` produces a fresh
``(model, data)`` pair so trials are fully independent.
"""

import numpy as np
import mujoco


# MJCF template. Obstacles are inserted by string substitution.
MJCF_TEMPLATE = """
<mujoco model="turtle_world">
  <compiler angle="radian"/>
  <option timestep="0.005" integrator="implicitfast" gravity="0 0 -9.81"/>
  <default>
    <geom friction="1.0 0.005 0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="20 20 0.1" rgba="0.85 0.9 0.85 1"/>

    <body name="turtle" pos="0 0 0.20">
      <joint type="free"/>
      <geom name="chassis" type="box" size="0.18 0.12 0.04" mass="2.0" rgba="0.2 0.4 0.85 1"/>

      <body name="left_wheel" pos="0 0.16 -0.10">
        <joint name="left_wheel" type="hinge" axis="0 1 0" damping="0.05"/>
        <geom type="cylinder" size="0.10 0.025" euler="1.5708 0 0" mass="0.3" rgba="0.1 0.1 0.1 1"/>
      </body>
      <body name="right_wheel" pos="0 -0.16 -0.10">
        <joint name="right_wheel" type="hinge" axis="0 1 0" damping="0.05"/>
        <geom type="cylinder" size="0.10 0.025" euler="1.5708 0 0" mass="0.3" rgba="0.1 0.1 0.1 1"/>
      </body>

      <body name="caster" pos="0.13 0 -0.15">
        <joint type="ball" damping="0.001"/>
        <geom type="sphere" size="0.05" mass="0.05"
              friction="0.1 0.001 0.001" rgba="0.5 0.5 0.5 1"/>
      </body>
    </body>

    <body name="target" pos="6 0 0.3">
      <geom type="sphere" size="0.3" rgba="0 1 0 0.4" contype="0" conaffinity="0"/>
    </body>

    {obstacles}
  </worldbody>

  <actuator>
    <velocity name="left_motor" joint="left_wheel" kv="3" forcerange="-15 15"/>
    <velocity name="right_motor" joint="right_wheel" kv="3" forcerange="-15 15"/>
  </actuator>
</mujoco>
"""


def random_obstacle_field(n=6, x_range=(1.0, 5.5), y_range=(-1.4, 1.4), seed=None):
    """Sample ``n`` obstacle positions uniformly in a rectangle in front of the robot.

    Args:
        n: Number of obstacles to sample.
        x_range: ``(min, max)`` x bounds.
        y_range: ``(min, max)`` y bounds.
        seed: RNG seed. Pass an int for reproducible layouts.

    Returns:
        list[tuple[float, float]]: List of ``(x, y)`` obstacle positions.
    """
    rng = np.random.default_rng(seed)
    xs = rng.uniform(*x_range, n)
    ys = rng.uniform(*y_range, n)
    return [(float(x), float(y)) for x, y in zip(xs, ys)]


def build_scene(obstacle_positions):
    """Compile a fresh MJCF scene with the given obstacle layout.

    Args:
        obstacle_positions: Iterable of ``(x, y)`` planar positions for each
            box obstacle.

    Returns:
        tuple: ``(model, data)`` ready to step with :func:`mujoco.mj_step`.
    """
    obstacle_xml = "\n    ".join(
        f'<body name="obs_{i}" pos="{x} {y} 0.15">'
        f'<geom type="box" size="0.10 0.10 0.15" rgba="0.85 0.25 0.25 1"/></body>'
        for i, (x, y) in enumerate(obstacle_positions)
    )
    xml = MJCF_TEMPLATE.format(obstacles=obstacle_xml)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data
