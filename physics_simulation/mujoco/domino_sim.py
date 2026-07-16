import numpy as np

import mujoco
from mujoco import viewer


def generate_domino_xml(n_dominos=25, spacing=0.05):
    xml = """
    <mujoco model="domino_line">
    <option timestep="0.001" gravity="0 0 -9.81"/>
    <default>
        <geom type="box" density="1500" friction="0.6 0.01 0.001" margin="0.001"/>
    </default>
    <worldbody>
        <!-- Ground -->
        <geom type="plane" size="10 10 0.1" rgba="0.2 0.2 0.2 1"/>
    """
    half_w = 0.01  # half width
    half_d = 0.005  # half depth
    half_h = 0.04  # half height

    for i in range(n_dominos):
        x = i * spacing
        tilt = 12 if i == 0 else 0  # First domino tilted
        xml += f"""
    <body name="domino{i}" pos="{x} 0 {half_h}" euler="0 {tilt} 0">
      <freejoint name="joint{i}"/>
      <geom size="{half_w} {half_d} {half_h}" rgba="0.9 0.4 0.3 1"/>
    </body>
"""
    xml += """
  </worldbody>
</mujoco>
"""
    return xml


# ---- Speed control -------------------------------------------------
speed_factor = 0.3  # start in slow-motion
key_speed_step = 0.5  # how much to change per press

# --- RUN MODEL ---
xml = generate_domino_xml(50)
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# === Give first domino an initial angular velocity (pitch forward) ===
body_id = model.body("domino0").id
joint_adr = model.body_jntadr[body_id]
dof_adr = model.jnt_dofadr[joint_adr]

# Set angular velocity around Y-axis (index 4 in qvel: [vx,vy,vz,wx,wy,wz])
data.qvel[dof_adr + 4] = 5.0  # wy = 2.0 rad/s → tips forward

# Optional: slightly reduce friction globally
model.geom_friction[:] = np.array([0.6, 0.01, 0.001])

# === Launch Viewer ===
with viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
