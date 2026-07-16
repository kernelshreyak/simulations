"""
Full, runnable MuJoCo box-rain simulation.

Requirements:
  pip install mujoco numpy

Run:
  python box_rain_sim.py
"""

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


def generate_box_rain_xml(
    n_boxes=60,
    z_min=2.0,
    z_max=12.0,
    xy_spread=1.0,
    size_range=(0.06, 0.18),
    seed=1,
):
    rng = np.random.default_rng(seed)

    def rand_color():
        return f"{rng.random():.2f} {rng.random():.2f} {rng.random():.2f} 1"

    bodies = []
    for _ in range(n_boxes):
        x = rng.uniform(-xy_spread, xy_spread)
        y = rng.uniform(-xy_spread, xy_spread)
        z = rng.uniform(z_min, z_max)

        sx = rng.uniform(*size_range)
        sy = rng.uniform(*size_range)
        sz = rng.uniform(*size_range)

        bodies.append(f"""
    <body pos="{x:.3f} {y:.3f} {z:.3f}">
      <joint type="free"/>
      <geom type="box"
            size="{sx:.3f} {sy:.3f} {sz:.3f}"
            rgba="{rand_color()}"
            friction="0.8 0.1 0.1"/>
    </body>
""")

    return f"""<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81" iterations="50"/>
  <compiler angle="degree"/>

  <worldbody>

    <light diffuse="0.6 0.6 0.6" pos="0 0 6" dir="0 0 -1"/>
    <light diffuse="0.4 0.4 0.4" pos="3 3 6" dir="-1 -1 -1"/>

    <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"
          friction="1.0 0.5 0.5"/>

    {"".join(bodies)}

  </worldbody>
</mujoco>
"""


def main():
    xml_path = Path("box_rain.xml")
    xml_path.write_text(
        generate_box_rain_xml(
            n_boxes=100,
            z_min=2.0,
            z_max=14.0,
            xy_spread=1.2,
            size_range=(0.05, 0.3),
            seed=4,
        )
    )

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Add small random initial velocities for extra chaos
    rng = np.random.default_rng(0)
    for i in range(model.nv):
        data.qvel[i] = rng.normal(scale=0.2)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 6.0
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -25

        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

            # real-time pacing
            dt = model.opt.timestep
            sleep = dt - (time.time() - step_start)
            if sleep > 0:
                time.sleep(sleep)


if __name__ == "__main__":
    main()
