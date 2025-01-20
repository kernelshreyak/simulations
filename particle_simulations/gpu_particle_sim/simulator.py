"""
This Python module simulates the movement and interaction of particles within specified boundary conditions using CUDA for parallel processing.

The key components include:
- Initializing particle positions based on boundary types such as spheres, cubes, and cuboids.
- Computing and updating particle positions and velocities using CUDA kernels, considering interactions such as repulsion/attraction and sticky interactions.
- Handling forces from gravity and boundary conditions to reflect particles within the domain.
- Simulating particle trajectories over a specified simulation time, with results saved for further analysis.
- Configuration and parameters are imported from an external configuration module.

Primary functions and their roles:
- update_particles: CUDA kernel to update positions, velocities, and handle interactions and boundary conditions.
- random_positions: Generates initial random positions for particles based on the boundary configuration.
- line_positions: Initializes particles along a line within a cube boundary.
- circle_positions: Initializes particles on the surface of a sphere.
- simulate_particles: Main simulation loop, updating particle states and storing trajectories.

CUDA usage ensures efficient computation by leveraging parallel processing capabilities for particle interactions.
"""

import numpy as np
import math
from numba import cuda
from simulation_config import REPULSIVE_CONSTANT,PARTICLE_MASS,NUM_PARTICLES,SIMULATION_TIME,TIME_STEP,BOUNCE_FACTOR_BOUNDARY,BOUNDARY_TYPE,BOUNDARY_PARAMS,THREADS_PER_BLOCK,ENABLE_GRAVITY,INTERACTION_TYPE

# CUDA Kernel for updating particle positions and velocities
@cuda.jit
def update_particles(positions, velocities, forces, num_particles, dt, enable_gravity, interaction_type, domain_type, domain_params):
    idx = cuda.grid(1)
    if idx >= num_particles:
        return

     # handle inter-particle interactions
    if interaction_type > 0:

        for j in range(num_particles):
            if idx != j:
                dx = positions[j, 0] - positions[idx, 0]
                dy = positions[j, 1] - positions[idx, 1]
                dz = positions[j, 2] - positions[idx, 2]
                dist = math.sqrt(dx**2 + dy**2 + dz**2 + 1e-6)  # Avoid division by zero

                if interaction_type == 1:
                    # Compute repulsive/attractive force (inverse-square law)
                    force_magnitude = -REPULSIVE_CONSTANT / (dist**2)
                    forces[idx, 0] += force_magnitude * dx / dist
                    forces[idx, 1] += force_magnitude * dy / dist
                    forces[idx, 2] += force_magnitude * dz / dist

                elif interaction_type == 2:
                    # Compute logic for "sticky interaction"
                    forces[idx, 0] = 0
                    forces[idx, 1] = 0
                    forces[idx, 2] = 0

    # Add gravity to the forces
    if enable_gravity:
        forces[idx, 1] += -9.8 * PARTICLE_MASS

    # Update velocities
    velocities[idx, 0] += forces[idx, 0] * dt / PARTICLE_MASS
    velocities[idx, 1] += forces[idx, 1] * dt / PARTICLE_MASS
    velocities[idx, 2] += forces[idx, 2] * dt / PARTICLE_MASS

    # Update positions
    positions[idx, 0] += velocities[idx, 0] * dt
    positions[idx, 1] += velocities[idx, 1] * dt
    positions[idx, 2] += velocities[idx, 2] * dt

    # Reset forces
    forces[idx, 0] = 0.0
    forces[idx, 1] = 0.0
    forces[idx, 2] = 0.0


    # Handle boundary interactions and boundary conditions
    if domain_type == 1:  # Sphere
        radius = domain_params[0]
        dist_from_origin = math.sqrt(positions[idx, 0]**2 + positions[idx, 1]**2 + positions[idx, 2]**2)
        if dist_from_origin > radius:
            if BOUNCE_FACTOR_BOUNDARY > 0:
                # Reflect velocity
                norm = dist_from_origin / radius
                positions[idx, 0] /= norm
                positions[idx, 1] /= norm
                positions[idx, 2] /= norm
                velocities[idx, 0] *= BOUNCE_FACTOR_BOUNDARY
                velocities[idx, 1] *= BOUNCE_FACTOR_BOUNDARY
                velocities[idx, 2] *= BOUNCE_FACTOR_BOUNDARY

    elif domain_type == 2:  # Cube
        bounds = domain_params[0]
        for dim in range(3):
            if abs(positions[idx, dim]) > bounds:
                if BOUNCE_FACTOR_BOUNDARY > 0:
                    positions[idx, dim] = math.copysign(bounds, positions[idx, dim])
                    velocities[idx, dim] *= BOUNCE_FACTOR_BOUNDARY

def random_positions(boundary_type, boundary_params, num_particles):
    positions = np.zeros((num_particles, 3), dtype=np.float32)
    if boundary_type == "sphere":
        radius = boundary_params[0]
        for i in range(num_particles):
            # Random points inside a sphere
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)
            r = radius * (np.random.rand() ** (1/3))  # To ensure uniform distribution inside the sphere
            positions[i] = r * np.array([np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)])

    elif boundary_type == "cube":
        bounds = boundary_params[0]
        for i in range(num_particles):
            positions[i] = np.random.uniform(-bounds, bounds, size=3)

    elif boundary_type == "cuboid":
        bounds = boundary_params
        for i in range(num_particles):
            positions[i] = np.random.uniform(-bounds[0], bounds[0]), np.random.uniform(-bounds[1], bounds[1]), np.random.uniform(-bounds[2], bounds[2])

    return positions


def line_positions(boundary_type, boundary_params, num_particles):
    positions = np.zeros((num_particles, 3), dtype=np.float32)
    if boundary_type == "cube":
        # Define two points on the line (in the cube region)
        p1 = np.array([-boundary_params[0], 0, 0], dtype=np.float32)
        p2 = np.array([boundary_params[0], 0, 0], dtype=np.float32)
        for i in range(num_particles):
            t = np.random.rand()  # Random parameter along the line
            positions[i] = (1 - t) * p1 + t * p2  # Linear interpolation between p1 and p2
    return positions


def circle_positions(boundary_type, boundary_params, num_particles):
    positions = np.zeros((num_particles, 3), dtype=np.float32)
    if boundary_type == "sphere":
        radius = boundary_params[0]
        for i in range(num_particles):
            # Random points on the surface of a sphere (circle in 3D)
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)
            positions[i] = radius * np.array([np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)])

    return positions


def simulate_particles():
    num_frames = int(SIMULATION_TIME / TIME_STEP)

    # Select position initialization based on boundary type
    if BOUNDARY_TYPE == "sphere":
        positions = random_positions(BOUNDARY_TYPE, [BOUNDARY_PARAMS["sphere"]["radius"]], NUM_PARTICLES)
    elif BOUNDARY_TYPE == "cube":
        positions = random_positions(BOUNDARY_TYPE, [BOUNDARY_PARAMS["cube"]["bounds"]], NUM_PARTICLES)
    elif BOUNDARY_TYPE == "cuboid":
        positions = random_positions(BOUNDARY_TYPE, [BOUNDARY_PARAMS["cuboid"]["bounds"]], NUM_PARTICLES)
    else:
        positions = random_positions(BOUNDARY_TYPE, [0.0], NUM_PARTICLES)  # Default case, no boundaries

    velocities = np.zeros((NUM_PARTICLES, 3), dtype=np.float32)
    forces = np.zeros((NUM_PARTICLES, 3), dtype=np.float32)
    trajectories = np.zeros((num_frames, NUM_PARTICLES, 3), dtype=np.float32)

    # Allocate GPU memory
    d_positions = cuda.to_device(positions)
    d_velocities = cuda.to_device(velocities)
    d_forces = cuda.to_device(forces)

    # Determine domain type and parameters
    domain_type = 0  # Default: no domain
    domain_params = np.array([0.0], dtype=np.float32)
    if BOUNDARY_TYPE == "sphere":
        domain_type = 1
        domain_params = np.array([BOUNDARY_PARAMS["sphere"]["radius"]], dtype=np.float32)
    elif BOUNDARY_TYPE == "cube":
        domain_type = 2
        domain_params = np.array([BOUNDARY_PARAMS["cube"]["bounds"]], dtype=np.float32)

    d_domain_params = cuda.to_device(domain_params)

    # CUDA kernel configuration
    blocks = (NUM_PARTICLES + THREADS_PER_BLOCK - 1) // THREADS_PER_BLOCK

    # Simulation loop
    for frame in range(num_frames):
        update_particles[blocks, THREADS_PER_BLOCK](
            d_positions, d_velocities, d_forces, NUM_PARTICLES, TIME_STEP,
            ENABLE_GRAVITY, INTERACTION_TYPE, domain_type, d_domain_params
        )
        cuda.synchronize()
        trajectories[frame] = d_positions.copy_to_host()
        print(f"Frame {frame + 1}/{num_frames} simulated.")

    np.save("trajectories.npy", trajectories)
    print("Simulation complete. Trajectories saved to 'trajectories.npy'.")


if __name__ == "__main__":
    simulate_particles()
