import pybullet as p
import time
import pybullet_data
import random

def run_simulation(simulation_time, num_boxes):
    # Start the physics simulation
    physics_client = p.connect(p.GUI)  # GUI mode

    # Set additional search path for pybullet data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set gravity for the simulation (negative z-axis for gravity)
    p.setGravity(0, 0, -9.8)

    # Load the plane from Bullet's data path
    plane_id = p.loadURDF("plane.urdf")

    # Create multiple boxes with random orientations stacked on top of each other
    box_height = 1  # Height of each box (meters)
    initial_height = 5  # Starting height for the first box
    boxes = []

    for i in range(num_boxes):
        # Random orientation for the box (using Euler angles)
        random_orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        
        # Position the box starting above the ground, each box above the previous one
        box_start_position = [0, 0, initial_height + i * box_height]

        # Create a box
        box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
        box_body = p.createMultiBody(baseMass=5, baseCollisionShapeIndex=box_collision_shape, 
                                     basePosition=box_start_position, baseOrientation=random_orientation)
        boxes.append(box_body)

    # Calculate the number of simulation steps based on the simulation time (60Hz steps)
    num_steps = int(simulation_time * 60)

    # Run the simulation
    for _ in range(num_steps):
        p.stepSimulation()
        time.sleep(.5/60.)  

    # Disconnect the simulation
    p.disconnect()

if __name__ == "__main__":
    # Simulation parameters
    simulation_time = 30  # Run the simulation for 10 seconds
    num_boxes = 5         # Number of boxes stacked on top of each other

    run_simulation(simulation_time, num_boxes)
