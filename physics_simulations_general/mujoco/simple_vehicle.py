import time

import mujoco
import mujoco.viewer

XML_PATH = "simple_vehicle.xml"


def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 4.5
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -25

        while viewer.is_running():
            # ===== Controls =====
            drive = 0.6  # rear wheel torque
            steer = 0.4  # radians (~23 deg)

            data.ctrl[0] = drive  # rear_left
            data.ctrl[1] = drive  # rear_right
            data.ctrl[2] = steer  # steer_fl
            data.ctrl[3] = steer  # steer_fr

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
