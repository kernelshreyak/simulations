import numpy as np

import mujoco
from mujoco import viewer

xml = """
<mujoco>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.2">
      <joint type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with viewer.launch_passive(model, data) as v:
    while v.is_running():
        data.ctrl[:] = np.array([0.5])  # spin slowly
        mujoco.mj_step(model, data)
        v.sync()
