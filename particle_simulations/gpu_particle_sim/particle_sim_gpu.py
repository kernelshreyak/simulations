import numpy as np
import math
from numba import cuda, float32

# Simulation Parameters
NUM_PARTICLES = 20000
SIMULATION_TIME = 30.0  # seconds
TIME_STEP = 0.01        # seconds
PARTICLE_MASS = 50.0     # kg
REPULSIVE_CONSTANT = 5.0  # Strength of the repulsive force
VIDEO_FPS = 30

# Configurable Options
ENABLE_GRAVITY = True
ENABLE_INTERPARTICLE_FORCE = True
DOMAIN_TYPE = "cube"  # Options: 'sphere', 'cube', 'cuboid', 'torus', 'cylinder', 'none'

DOMAIN_PARAMS = {
    "sphere": {"radius": 4.0},
    "cube": {"bounds": 5.0},  # x, y, z bounds
    "cuboid": {"bounds": (5.0, 5.0, 5.0)},  # x, y, z bounds
    "torus": {"major_radius": 4.0, "minor_radius": 1.0},
    "cylinder": {"radius": 3.0, "height": 6.0}
}

# CUDA Kernel for updating particle positions and velocities
@cuda.jit
def update_particles(positions, velocities, forces, num_particles, dt, enable_gravity, enable_force, domain_type, domain_params):
    idx = cuda.grid(1)
    if idx >= num_particles:
        return

    # Update forces between particles
    if enable_force:
        for j in range(num_particles):
            if idx != j:
                dx = positions[j, 0] - positions[idx, 0]
                dy = positions[j, 1] - positions[idx, 1]
                dz = positions[j, 2] - positions[idx, 2]
                dist = math.sqrt(dx**2 + dy**2 + dz**2 + 1e-6)  # Avoid division by zero

                # Compute repulsive force (inverse-square law)
                force_magnitude = REPULSIVE_CONSTANT / (dist**2)
                forces[idx, 0] += force_magnitude * dx / dist
                forces[idx, 1] += force_magnitude * dy / dist
                forces[idx, 2] += force_magnitude * dz / dist

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

    # Handle domain constraints
    if domain_type == 1:  # Sphere
        radius = domain_params[0]
        dist_from_origin = math.sqrt(positions[idx, 0]**2 + positions[idx, 1]**2 + positions[idx, 2]**2)
        if dist_from_origin > radius:
            # Reflect velocity
            norm = dist_from_origin / radius
            positions[idx, 0] /= norm
            positions[idx, 1] /= norm
            positions[idx, 2] /= norm
            velocities[idx, 0] *= -1
            velocities[idx, 1] *= -1
            velocities[idx, 2] *= -1
    elif domain_type == 2:  # Cube
        bounds = domain_params[0]
        for dim in range(3):
            if abs(positions[idx, dim]) > bounds:
                positions[idx, dim] = math.copysign(bounds, positions[idx, dim])
                velocities[idx, dim] *= -1


def simulate_particles():
    num_frames = int(SIMULATION_TIME / TIME_STEP)
    positions = np.random.rand(NUM_PARTICLES, 3).astype(np.float32) * 2 - 1
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
    if DOMAIN_TYPE == "sphere":
        domain_type = 1
        domain_params = np.array([DOMAIN_PARAMS["sphere"]["radius"]], dtype=np.float32)
    elif DOMAIN_TYPE == "cube":
        domain_type = 2
        domain_params = np.array([DOMAIN_PARAMS["cube"]["bounds"]], dtype=np.float32)

    d_domain_params = cuda.to_device(domain_params)

    # CUDA kernel configuration
    threads_per_block = 256
    blocks = (NUM_PARTICLES + threads_per_block - 1) // threads_per_block

    # Simulation loop
    for frame in range(num_frames):
        update_particles[blocks, threads_per_block](
            d_positions, d_velocities, d_forces, NUM_PARTICLES, TIME_STEP,
            ENABLE_GRAVITY, ENABLE_INTERPARTICLE_FORCE, domain_type, d_domain_params
        )
        cuda.synchronize()
        trajectories[frame] = d_positions.copy_to_host()
        print(f"Frame {frame + 1}/{num_frames} simulated.")

    np.save("trajectories.npy", trajectories)
    print("Simulation complete. Trajectories saved to 'trajectories.npy'.")


if __name__ == "__main__":
    simulate_particles()
