#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 21 11:24:45 2026

@author: shreyak
"""

import numpy as np
import pyvista as pv

# Create grid
x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)
x, y = np.meshgrid(x, y)

# Initial surface
z = np.sin(np.sqrt(x**2 + y**2))
grid = pv.StructuredGrid(x, y, z)

# Plotter
plotter = pv.Plotter()
actor = plotter.add_mesh(
    grid,
    cmap="viridis",
    show_edges=False,
    smooth_shading=True
)

plotter.add_axes()
plotter.add_text("PyVista Wave Animation", font_size=10)

# Animation update
def update(frame):
    z = np.sin(np.sqrt(x**2 + y**2) - frame * 0.2)
    grid.points[:, 2] = z.ravel()
    return grid

# Animate
#plotter.open_gif("wave.gif")  # optional: saves animation
for i in range(60):
    update(i)
    plotter.write_frame()

plotter.close()

# Show final interactive window
plotter = pv.Plotter()
plotter.add_mesh(grid, cmap="plasma", smooth_shading=True)
plotter.show()