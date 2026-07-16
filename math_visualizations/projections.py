import numpy as np
import matplotlib.pyplot as plt

# Define the 2D dataset
data = np.array([
    [-2, 30],
    [3, 5],
    [4, 15],
    [5, 6],
    [6, 7]
])

# Define the direction of the line (vector)
line_direction = np.array([1, 1])

# Normalize the direction vector
line_direction = line_direction / np.linalg.norm(line_direction)

# Project the points onto the line
projections = data.dot(line_direction)

# Calculate the coordinates of the projected points on the line
projected_points = np.outer(projections, line_direction)

# Define a range for the line to ensure it extends sufficiently
line_magnitude = np.linalg.norm([np.max(data[:, 0]) - np.min(data[:, 0]), np.max(data[:, 1]) - np.min(data[:, 1])])
line_start = -line_magnitude * line_direction
line_end = line_magnitude * line_direction
line_points = np.array([line_start, line_end])

# Plot the dataset, line, and projected points
plt.figure(figsize=(8, 6))

# Plot original data points
plt.scatter(data[:, 0], data[:, 1], color='blue', label='Original Data Points')

# Plot the line
plt.plot(line_points[:, 0], line_points[:, 1], color='green', label='Projection Line')

# Plot the projected points
plt.scatter(projected_points[:, 0], projected_points[:, 1], color='red', label='Projected Points')

# Add lines from original points to their projections
for i in range(data.shape[0]):
    plt.plot([data[i, 0], projected_points[i, 0]], [data[i, 1], projected_points[i, 1]], color='gray', linestyle='--')

# Additional plot settings
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Projection of 2D Data Points onto a Line')
plt.legend()
plt.grid(True)

# Show plot
plt.show()
