
# Simulation Parameters
NUM_PARTICLES: int = 2000
SIMULATION_TIME = 20.0  # seconds
TIME_STEP:float = 0.01        # seconds
PARTICLE_MASS:float = 5.0     # kg
REPULSIVE_CONSTANT:float = -5.0  # Strength of the repulsive force (negative is for attraction)

# Configurable Options
ENABLE_GRAVITY: bool = False
INTERACTION_TYPE = 1
BOUNDARY_TYPE: str = "cube"  # Options: 'sphere', 'cube', 'cuboid', 'torus', 'cylinder', 'none'

# Boundary conditions
BOUNCE_FACTOR_BOUNDARY: float = 1.5

BOUNDARY_PARAMS = {
    "sphere": {"radius": 4.0},
    "cube": {"bounds": 5.0},  # x, y, z bounds
    "cuboid": {"bounds": (5.0, 5.0, 5.0)},  # x, y, z bounds
    "torus": {"major_radius": 4.0, "minor_radius": 1.0},
    "cylinder": {"radius": 3.0, "height": 6.0}
}

# GPU params
THREADS_PER_BLOCK: int = 256
