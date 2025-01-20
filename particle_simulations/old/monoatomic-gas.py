import open3d as o3d
import numpy as np
import time

def create_particle(radius, center):
    """ Create a particle with given radius and center. """
    particle = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    particle.translate(center)
    particle.paint_uniform_color([0,0,0.8])
    return particle

# defines the logic for the equation of motion of the particle for various scenarios

# fully random speed for each time step
def get_motion_random() -> np.ndarray:
    random_x = np.random.uniform(0, 1)
    random_y = np.random.uniform(0,1)
    random_z = np.random.uniform(0, 2)
    return np.array([random_x,random_y,random_z])

def move_particles(vis,particles,motion_type:str):
    dt = 0.1
    i = 0
    if motion_type == "random":
        for particle in particles:
            total_speed = get_motion_random()*dt
            particle.translate(total_speed, relative=True)
            vis.update_geometry(particle)  # Update the geometry
            i += 1
    elif motion_type == "static":
        return
    else:
        raise Exception("Invalid motion type")

def generate_initial_state(volume, num_points,init_type:str):
    if init_type == "random":
        # Generate random angles and distance
        phi = np.random.uniform(0, 2 * np.pi, num_points)
        theta = np.arccos(1 - 2 * np.random.uniform(0, 1, num_points))
        r = volume * np.cbrt(np.random.uniform(0, 1, num_points))

        # Convert spherical coordinates to Cartesian coordinates
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)

        # Stack the coordinates into a single array
        points = np.column_stack((x, y, z))
        return points
    else:
        raise Exception("Invalid initial state config")

def main():
    N = 100  # Number of particles
    r = 0.1  # Radius of each particle
    V = 20  #volume of initial enclosure
    initial_positions = generate_initial_state(V,N,"random")  # Initial positions of particles

    # Create particles
    particles = [create_particle(r, pos) for pos in initial_positions]

    # Visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Particle Simulation', width=1600, height=900)

    for particle in particles:
        vis.add_geometry(particle)

    for _ in range(100):
        move_particles(vis,particles,"random")  
        # move_particles(vis,particles,"static")  
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)  # Adjust for desired speed
        
    vis.destroy_window()

if __name__ == "__main__":
    main()
