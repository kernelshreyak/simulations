import numpy as np
import pyvista as pv
from fipy import (
    CellVariable,
    ConvectionTerm,
    DiffusionTerm,
    FaceVariable,
    Grid2D,
    TransientTerm,
)
from tqdm.auto import tqdm

# =====================================================
# CONFIG
# =====================================================

ROOM_WIDTH = 5.0
ROOM_HEIGHT = 4.0

NX = 80
NY = 60

DT = 0.1
STEPS = 500

FRAME_SKIP = 5

# =====================================================
# MESH
# =====================================================

mesh = Grid2D(
    dx=ROOM_WIDTH / NX,
    dy=ROOM_HEIGHT / NY,
    nx=NX,
    ny=NY,
)

# =====================================================
# TEMPERATURE FIELD
# =====================================================

T = CellVariable(
    name="Temperature",
    mesh=mesh,
    value=25.0,
)

# =====================================================
# BOUNDARY CONDITIONS
# =====================================================

face_y = mesh.faceCenters[1]

# Sun-heated section on left wall

sun_wall = mesh.facesLeft & (face_y > 1.0) & (face_y < 3.0)

T.constrain(
    50.0,
    where=sun_wall,
)

# Remaining left wall

cool_left = mesh.facesLeft & ~sun_wall

T.constrain(
    25.0,
    where=cool_left,
)

# Right wall

T.constrain(
    25.0,
    where=mesh.facesRight,
)

# Sun-heated roof

T.constrain(
    45.0,
    where=mesh.facesTop,
)

# Floor

T.constrain(
    25.0,
    where=mesh.facesBottom,
)

# =====================================================
# OCCUPANT
# =====================================================

person_x = 3.0
person_y = 1.5

person_region = ((mesh.x - person_x) ** 2 + (mesh.y - person_y) ** 2) < 0.15**2

# =====================================================
# AIRFLOW
# =====================================================

# Simple room circulation:
#
# Ceiling  ---->
# Right wall ↓
# Floor   <----
# Left wall ↑

fx = mesh.faceCenters[0]
fy = mesh.faceCenters[1]

u = 1.5 * np.sin(np.pi * fy / ROOM_HEIGHT)

v = -0.8 * np.sin(np.pi * fx / ROOM_WIDTH)

velocity = FaceVariable(
    mesh=mesh,
    rank=1,
    value=(u, v),
)

# =====================================================
# PHYSICS
# =====================================================

alpha = 0.01

eq = TransientTerm() == DiffusionTerm(coeff=alpha) - 0.25 * ConvectionTerm(
    coeff=velocity
)

# =====================================================
# PYVISTA GRID
# =====================================================

x = np.linspace(
    0,
    ROOM_WIDTH,
    NX,
)

y = np.linspace(
    0,
    ROOM_HEIGHT,
    NY,
)

X, Y = np.meshgrid(
    x,
    y,
    indexing="ij",
)

grid = pv.StructuredGrid(
    X,
    Y,
    np.zeros_like(X),
)

field = T.value.reshape(
    (NX, NY),
    order="F",
)

grid["Temperature"] = field.flatten(order="F")

# =====================================================
# VIEWER
# =====================================================

plotter = pv.Plotter()

plotter.add_mesh(
    grid,
    scalars="Temperature",
    cmap="inferno",
    clim=[25, 50],
    show_edges=False,
)

# Occupant marker

plotter.add_points(
    np.array([[person_x, person_y, 0.0]]),
    color="cyan",
    point_size=15,
    render_points_as_spheres=True,
)

plotter.add_text(
    "Room Temperature",
    font_size=12,
)

plotter.view_xy()

plotter.show(
    interactive_update=True,
    auto_close=False,
)

# =====================================================
# SOLVER
# =====================================================

for step in tqdm(
    range(STEPS),
    desc="Simulation",
    unit="step",
):
    # Occupant heat source

    T.setValue(
        37.0,
        where=person_region,
    )

    eq.solve(
        var=T,
        dt=DT,
    )

    if step % FRAME_SKIP == 0:
        field = T.value.reshape(
            (NX, NY),
            order="F",
        )

        grid["Temperature"] = field.flatten(order="F")

        plotter.add_text(
            f"Step {step}",
            name="step_text",
            font_size=10,
        )

        plotter.render()

print("Finished")

plotter.show()
