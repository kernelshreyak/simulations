import time

import mujoco
import mujoco.viewer
import numpy as np

# --- 1. MuJoCo XML Model (MJCF) ---
# This XML defines the physics world: a ground plane and a single sphere (ball).
# The key for the bounce effect is the solref="0.02 0.95" attribute,
# where 0.95 is the coefficient of restitution (bounciness).
xml_model = """
<mujoco model="bouncing_ball">

  <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>

  <default>
    <!-- Define the default properties for the sphere and its interaction -->
    <!-- solref is key: [time_constant (s), stiffness_ratio (unitless)].
         The second value, 0.95, sets the coefficient of restitution. -->
    <geom type="sphere" size="0.1" density="1000" rgba="0.1 0.2 0.8 1"
          friction="0.9"
          solimp="0.99 0.99 0.01" solref="0.02 0.95"/>
  </default>

  <!-- Simulation options: Gravity is set to standard Earth gravity (m/s^2) -->
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <!-- Ground plane at Z=0. Size controls the checkerboard pattern repeat, not physical size. -->
    <geom name="ground" type="plane" pos="0 0 0" size="1 1 0.1"
          rgba="0.5 0.5 0.5 1" condim="3"/>

    <!-- Ball body: positioned 1.0 meter above the ground (pos="0 0 1") -->
    <body name="ball_body" pos="0 0 1">
      <!-- 'free' joint allows the body to move and rotate freely in 3D space (6 DOFs) -->
      <joint type="free"/>
      <!-- The geometry inherits default properties defined above -->
      <geom name="ball" pos="0 0 0"/>
    </body>

  </worldbody>
</mujoco>
"""

# --- 2. Load Model and Data ---
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

# --- 3. Simulation Parameters ---
# The simulation will run for 10 seconds of wall clock time.
SIMULATION_DURATION = 10.0  # seconds

# --- 4. Interactive Simulation Loop ---
print(f"Starting MuJoCo interactive simulation for {SIMULATION_DURATION} seconds.")
print(f"Gravity: 9.81 m/s^2")
print(f"Coefficient of Restitution: 0.95 (unitless)")

start_time = time.time()

# launch_passive opens a non-blocking viewer window.
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set the camera to a nice viewing angle
    viewer.cam.distance = 2.0  # meters
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -15
    viewer.cam.lookat = [0.0, 0.0, 0.5]  # meters

    # Loop until the wall clock duration has passed
    while viewer.is_running() and time.time() - start_time < SIMULATION_DURATION:
        # Step the physics simulation forward
        mujoco.mj_step(model, data)

        # Update viewer to show the new state
        viewer.sync()

        # Optional: Add a short pause for smoother visualization if the simulation runs too fast
        # time.sleep(0.001)

# --- 5. Final State Information ---
final_pos = data.body("ball_body").xpos
print("\nSimulation Finished.")
print(
    f"Final ball position (x, y, z): ({final_pos[0]:.3f} m, {final_pos[1]:.3f} m, {final_pos[2]:.3f} m)"
)
