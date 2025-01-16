import numpy as np
import open3d as o3d
import time

def visualize_point_cloud(npy_file="trajectories.npy", frame_delay=0.05):
    """
    Visualize particle positions as a point cloud using Open3D.

    Args:
        npy_file (str): Path to the .npy file containing particle trajectories.
        frame_delay (float): Delay between frames for smoother visualization.
    """
    # Load particle trajectories
    trajectories = np.load(npy_file)
    num_frames, num_particles, _ = trajectories.shape

    # Compute velocities (magnitude of the velocity vector between consecutive frames)
    velocities = np.zeros((num_frames, num_particles))
    for frame in range(1, num_frames):
        velocities[frame] = np.linalg.norm(trajectories[frame] - trajectories[frame - 1], axis=-1)

    # Normalize velocity magnitudes for color mapping
    max_velocity = np.max(velocities)
    if max_velocity > 0:
        velocities /= max_velocity  # Normalize to range [0, 1]

    # Initialize Open3D visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Particle Simulation", width=1024, height=768)

    # Create a point cloud object
    point_cloud = o3d.geometry.PointCloud()

    # Add initial positions and default colors
    initial_positions = trajectories[0]
    point_cloud.points = o3d.utility.Vector3dVector(initial_positions)
    default_colors = np.zeros((num_particles, 3))  # Black by default
    default_colors[:, 2] = 1.0  # Blue color
    point_cloud.colors = o3d.utility.Vector3dVector(default_colors)
    vis.add_geometry(point_cloud)

    # Set camera view
    vis.get_view_control().set_zoom(0.8)

    # Simulation playback
    for frame in range(num_frames):
        # Update point cloud positions
        positions = trajectories[frame]
        point_cloud.points = o3d.utility.Vector3dVector(positions)

        # Update point cloud colors based on velocity
        velocity_colors = np.zeros((num_particles, 3))
        velocity_colors[:, 0] = velocities[frame]  # Map velocity to red channel (higher velocity -> red)
        velocity_colors[:, 2] = 1.0 - velocities[frame]  # Low velocity -> blue
        point_cloud.colors = o3d.utility.Vector3dVector(velocity_colors)

        # Update visualization
        vis.update_geometry(point_cloud)
        vis.poll_events()
        vis.update_renderer()

        # Add delay for visualization
        time.sleep(frame_delay)

    vis.destroy_window()


if __name__ == "__main__":
    # Replace 'trajectories.npy' with your actual file path
    visualize_point_cloud("trajectories.npy", frame_delay=0.1)
