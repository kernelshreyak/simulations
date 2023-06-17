import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Size of the grid
GRID_SIZE = 80

# Initialize the grid randomly
grid = np.random.choice([0, 1], size=(GRID_SIZE, GRID_SIZE), p=[0.8, 0.2])

# Set N cells as alive on an empty grid
# grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
# sector_start_x = 20
# sector_start_y = 20
# grid[sector_start_x,sector_start_y] = 1
# grid[sector_start_x+1,sector_start_y] = 1
# grid[sector_start_x+2,sector_start_y] = 1
# grid[sector_start_x+2,sector_start_y+1] = 1
# grid[sector_start_x+2,sector_start_y+2] = 1

# Create a figure and axis
fig, ax = plt.subplots()

# Function to update the grid for each iteration
def update(frame):
    global grid
    new_grid = grid.copy()

    # Iterate through each cell
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            # Count the number of live neighbors
            neighbors = (
                grid[(i - 1) % GRID_SIZE, (j - 1) % GRID_SIZE]
                + grid[(i - 1) % GRID_SIZE, j]
                + grid[(i - 1) % GRID_SIZE, (j + 1) % GRID_SIZE]
                + grid[i, (j - 1) % GRID_SIZE]
                + grid[i, (j + 1) % GRID_SIZE]
                + grid[(i + 1) % GRID_SIZE, (j - 1) % GRID_SIZE]
                + grid[(i + 1) % GRID_SIZE, j]
                + grid[(i + 1) % GRID_SIZE, (j + 1) % GRID_SIZE]
            )

            # Apply the rules of Conway's Game of Life
            if grid[i, j] == 1:
                if neighbors < 2 or neighbors > 3:
                    new_grid[i, j] = 0
            else:
                if neighbors == 3:
                    new_grid[i, j] = 1

    # Update the grid
    mat.set_array(new_grid)
    grid = new_grid

# Create a plot with the initial grid
mat = ax.matshow(grid, cmap='binary')

# Create an animation
ani = animation.FuncAnimation(fig, update, interval=200, save_count=50)

# plt.rcParams['figure.figsize'] = [20, 20]

# Display the animation
plt.show()
