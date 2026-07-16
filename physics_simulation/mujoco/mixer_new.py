# mixer_grinder_fixed.py
import mujoco
import numpy as np
from mujoco import viewer

# ------------------- CONFIG -------------------
N_BALLS = 30
BLADE_RPM = 20
BLADE_RADIUS = 0.12
DROP_HEIGHT_MIN = 0.40
DROP_HEIGHT_MAX = 0.80
BALL_RADIUS = 0.008

# container (cube)
BOX_HALF_X = 0.18
BOX_HALF_Y = 0.18
BOX_HALF_Z = 0.30
WALL_THICKNESS = 0.02
# ---------------------------------------------


def generate_mixer_xml():
    xml = f"""
<mujoco model="mixer_grinder">
  <compiler angle="radian"/>
  <option timestep="0.0005" gravity="0 0 -9.81" iterations="200" solver="Newton" tolerance="1e-10"/>

  <!-- ---------- LIGHTING ---------- -->
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global offwidth="1280" offheight="720"/>
  </visual>

  <default>
    <geom friction="0.7 0.05 0.001" density="7800" rgba="0.7 0.7 0.7 1"/>
    <joint damping="0.01"/>
  </default>

  <worldbody>
    <!-- ---------- LIGHTS ---------- -->
    <light name="top"   pos=" 0  0 2" dir=" 0  0 -1" diffuse="0.9 0.9 0.9"/>
    <light name="left"  pos="-2  0 2" dir=" 1  0 -1" diffuse="0.5 0.5 0.6"/>
    <light name="right" pos=" 2  0 2" dir="-1  0 -1" diffuse="0.5 0.6 0.5"/>

    <!-- ---------- CONTAINER WALLS ---------- -->
    <!-- front -->
    <geom type="box" size="{WALL_THICKNESS} {BOX_HALF_Y * 2} {BOX_HALF_Z * 2}"
          pos="{BOX_HALF_X + WALL_THICKNESS / 2} 0 {BOX_HALF_Z}" rgba="0.35 0.35 0.35 0.8"/>
    <!-- back -->
    <geom type="box" size="{WALL_THICKNESS} {BOX_HALF_Y * 2} {BOX_HALF_Z * 2}"
          pos="-{BOX_HALF_X + WALL_THICKNESS / 2} 0 {BOX_HALF_Z}" rgba="0.35 0.35 0.35 0.8"/>
    <!-- left -->
    <geom type="box" size="{BOX_HALF_X * 2 + WALL_THICKNESS * 2} {WALL_THICKNESS} {BOX_HALF_Z * 2}"
          pos="0 -{BOX_HALF_Y + WALL_THICKNESS / 2} {BOX_HALF_Z}" rgba="0.35 0.35 0.35 0.8"/>
    <!-- right -->
    <geom type="box" size="{BOX_HALF_X * 2 + WALL_THICKNESS * 2} {WALL_THICKNESS} {BOX_HALF_Z * 2}"
          pos="0 {BOX_HALF_Y + WALL_THICKNESS / 2} {BOX_HALF_Z}" rgba="0.35 0.35 0.35 0.8"/>

    <!-- ---------- FLOOR ---------- -->
    <geom type="box"
          size="{BOX_HALF_X * 2 + WALL_THICKNESS * 2} {BOX_HALF_Y * 2 + WALL_THICKNESS * 2} {WALL_THICKNESS}"
          pos="0 0 {WALL_THICKNESS / 2}"
          rgba="0.4 0.4 0.4 0.9"
          contype="1" conaffinity="1"/>

    <!-- ---------- BLADE (HIGH-RESOLUTION CYLINDER + THIN HUB) ---------- -->
    <body name="blade" pos="0 0 {WALL_THICKNESS + 0.03}">
      <joint name="blade_joint" type="hinge" axis="0 0 1" damping="0.01"/>

      <!-- Main rotating blades as high-res cylinder for better sphere collision -->
      <geom type="cylinder" size="{BLADE_RADIUS} 0.005"
            rgba="1 0.3 0.3 1" contype="1" conaffinity="1" group="1"/>

      <!-- Thin hub in center (optional, for visual) -->
      <geom type="cylinder" size="0.02 0.015"
            pos="0 0 0" rgba="0.8 0.2 0.2 1" contype="1" conaffinity="1"/>
    </body>

    <!-- ---------- DROP ZONE ---------- -->
    <site name="drop_zone" pos="0 0 0.6" size="0.12" rgba="0 0 0 0"/>
"""
    # ---- 300 balls ----
    rng = np.random.default_rng(42)
    for i in range(N_BALLS):
        x = rng.uniform(-BOX_HALF_X * 0.8, BOX_HALF_X * 0.8)
        y = rng.uniform(-BOX_HALF_Y * 0.8, BOX_HALF_Y * 0.8)
        z = rng.uniform(DROP_HEIGHT_MIN, DROP_HEIGHT_MAX)
        xml += f"""
    <body name="ball{i}" pos="{x} {y} {z}">
      <freejoint/>
      <geom type="sphere" size="{BALL_RADIUS}" rgba="0.6 0.6 0.8 1"
            contype="1" conaffinity="1" margin="0.001"/>
    </body>
"""
    xml += """
  </worldbody>

  <actuator>
    <motor name="blade_motor" joint="blade_joint" gear="1"/>
  </actuator>
</mujoco>
"""
    return xml


# ------------------- BUILD & RUN -------------------
xml = generate_mixer_xml()
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# ---- spin blade ----
blade_angular_vel = BLADE_RPM * 2 * np.pi / 60
motor_id = model.actuator("blade_motor").id
data.ctrl[motor_id] = blade_angular_vel

# ---- slow-motion ----
speed_factor = 0.03
with viewer.launch_passive(model, data) as v:
    v.title = f"Mixer Grinder | {BLADE_RPM} RPM | {N_BALLS} balls | {speed_factor:.2f}×"
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync(model.opt.timestep / speed_factor)
