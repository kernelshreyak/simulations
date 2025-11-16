# slow_sync_demo.py
import mujoco
import numpy as np
from mujoco import viewer

# -------------------------------------------------
# 1. Build a tiny model (just one falling ball)
# -------------------------------------------------
xml = """
<mujoco model="demo">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1" rgba="0.3 0.3 0.3 1"/>
    <body pos="0 0 10">
      <freejoint/>
      <geom type="sphere" size="0.05" rgba="0.8 0.2 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# -------------------------------------------------
# 2. Speed control (change this line!)
# -------------------------------------------------
speed_factor = 1  # <-- 0.2 = 5x slower, 0.5 = 2x slower, 1.0 = real-time
# -------------------------------------------------

with viewer.launch_passive(model, data) as v:
    v.title = f"Speed: {speed_factor:.2f}x"

    while v.is_running():
        mujoco.mj_step(model, data)

        # ---- CORRECT SLOW-DOWN FORMULA ----
        v.sync(model.opt.timestep / speed_factor)
