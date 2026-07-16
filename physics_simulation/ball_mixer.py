import math
import time

import pybullet as p
import pybullet_data


def create_circular_container(
    radius=2.0, height=1.0, wall_thickness=0.1, num_segments=20
):
    """Create a circular container using multiple wall segments arranged in a circle."""
    walls = []
    # Define a transparent color for the walls (e.g., light blue with 50% transparency)
    wall_color = [0.5, 0.8, 1.0, 0.5]  # RGBA

    for i in range(num_segments):
        angle = (2 * math.pi / num_segments) * i
        next_angle = (2 * math.pi / num_segments) * ((i + 1) % num_segments)
        x1 = radius * math.cos(angle)
        y1 = radius * math.sin(angle)
        x2 = radius * math.cos(next_angle)
        y2 = radius * math.sin(next_angle)

        # Midpoint between the two points
        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2

        # Length and orientation of the wall segment
        wall_length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        wall_orientation = math.atan2(y2 - y1, x2 - x1)

        # Create wall collision shape
        wall_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[wall_length / 2, wall_thickness / 2, height / 2],
        )

        # Wall orientation quaternion
        orientation_quat = p.getQuaternionFromEuler([0, 0, wall_orientation])

        # Create wall visual shape with correct scaling and color
        wall_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[wall_length / 2, wall_thickness / 2, height / 2],
            rgbaColor=wall_color,
            specularColor=[0.5, 0.5, 0.5],
        )

        # Create wall multi-body
        wall = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=wall_collision_shape,
            baseVisualShapeIndex=wall_visual_shape,
            basePosition=[mx, my, height / 2],
            baseOrientation=orientation_quat,
        )

        walls.append(wall)
    return walls


def create_rotating_blade():
    """Create a rotating blade attached to the ground via a revolute joint."""
    # Base (static)
    base_mass = 0
    base_collision_shape = -1  # No collision shape
    base_visual_shape = -1
    base_position = [0, 0, 0]
    base_orientation = [0, 0, 0, 1]

    # Blade properties
    blade_mass = 50
    blade_collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX, halfExtents=[1.5, 0.1, 0.1]
    )
    blade_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX, halfExtents=[1.5, 0.1, 0.1], rgbaColor=[1, 0, 0, 1]
    )
    blade_position = [0, 0, 0.1]  # Relative to base
    blade_orientation = [0, 0, 0, 1]

    # Joint properties
    joint_type = [p.JOINT_REVOLUTE]
    joint_axis = [[0, 0, 1]]  # Rotation around Z-axis

    # Create multi-body with blade as a link
    blade_body = p.createMultiBody(
        baseMass=base_mass,
        baseCollisionShapeIndex=base_collision_shape,
        baseVisualShapeIndex=base_visual_shape,
        basePosition=base_position,
        baseOrientation=base_orientation,
        linkMasses=[blade_mass],
        linkCollisionShapeIndices=[blade_collision_shape],
        linkVisualShapeIndices=[blade_visual_shape],
        linkPositions=[blade_position],
        linkOrientations=[blade_orientation],
        linkInertialFramePositions=[[0, 0, 0]],
        linkInertialFrameOrientations=[[0, 0, 0, 1]],
        linkParentIndices=[0],
        linkJointTypes=joint_type,
        linkJointAxis=joint_axis,
    )
    return blade_body


def run_simulation(num_spheres=250, simulation_time=10):
    """Run the physics simulation with the rotating blade and falling spheres."""
    # Start physics client
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # Load plane (ground)
    plane_id = p.loadURDF("plane.urdf")

    # Create circular container with transparent walls
    create_circular_container(height=8, radius=1.7)

    # Create rotating blade
    blade_body = create_rotating_blade()

    # Create spheres that will fall into the container from the center
    sphere_radius = 0.1
    sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
    spheres = []
    for i in range(num_spheres):
        x = 0  # Center position
        y = 0  # Center position
        z = 2 + i * (2 * sphere_radius)  # Stack spheres vertically to avoid overlap
        sphere_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[0, 0, 1, 1]
        )
        sphere = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=sphere_collision_shape,
            baseVisualShapeIndex=sphere_visual_shape,
            basePosition=[x, y, z],
            baseOrientation=[0, 0, 0, 1],
        )
        spheres.append(sphere)

    # Simulation loop
    num_steps = int(simulation_time * 240)  # 240 steps per second
    for _ in range(num_steps):
        # Rotate the blade
        p.setJointMotorControl2(
            bodyUniqueId=blade_body,
            jointIndex=0,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=10000,  # radians per second
            force=5000,
        )
        p.stepSimulation()
        time.sleep(1.0 / 240.0)  # Real-time simulation

    # Disconnect the simulation
    p.disconnect()


if __name__ == "__main__":
    # Set parameters
    num_spheres = 100  # Number of spheres to drop
    simulation_time = 30  # Duration of the simulation in seconds

    run_simulation(num_spheres, simulation_time)
