import matplotlib
import numpy as np
from fipy import (
    CellVariable,
    DiffusionTerm,
    Grid2D,
    TransientTerm,
)
from tqdm.auto import tqdm

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

# =====================================================
# CONFIG
# =====================================================

ROOM_WIDTH = 5.0
ROOM_HEIGHT = 4.0

NX = 200
NY = 150

DT = 0.1
STEPS = 2000

SNAPSHOT_EVERY = 20

COOLER_START_STEP = 3000

AMBIENT_TEMP = 25.0
SUN_WALL_TEMP = 50.0
ROOF_TEMP = 45.0
PERSON_TEMP = 37.0
COOLER_TEMP = 18.0

ALPHA = 0.015

ROOM_MIXING = 0.015
JET_MIXING = 0.08

VIDEO_FILE = "room_simulation.mp4"

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
# TEMPERATURE
# =====================================================

T = CellVariable(
    name="Temperature",
    mesh=mesh,
    value=AMBIENT_TEMP,
)

# =====================================================
# BOUNDARIES
# =====================================================

face_y = mesh.faceCenters[1]

sun_wall = mesh.facesLeft & (face_y > 1.0) & (face_y < 3.0)

T.constrain(
    SUN_WALL_TEMP,
    where=sun_wall,
)

cool_left = mesh.facesLeft & ~sun_wall

T.constrain(
    AMBIENT_TEMP,
    where=cool_left,
)

T.constrain(
    AMBIENT_TEMP,
    where=mesh.facesRight,
)

T.constrain(
    ROOF_TEMP,
    where=mesh.facesTop,
)

T.constrain(
    AMBIENT_TEMP,
    where=mesh.facesBottom,
)

# =====================================================
# ROOM FEATURES
# =====================================================

x = mesh.cellCenters[0]
y = mesh.cellCenters[1]

# Person

person_x = 3.0
person_y = 1.5

person_region = ((x - person_x) ** 2 + (y - person_y) ** 2) < 0.12**2

# Cooler

cooler_region = (x < 0.7) & (y > 2.85) & (y < 3.55)

# Jet

jet_region = (x > 0.55) & (x < 4.0) & (y > 2.65 - 0.18 * x) & (y < 3.55 - 0.18 * x)

# =====================================================
# NUMPY MASKS
# =====================================================

Xc = np.asarray(x).reshape(
    (NX, NY),
    order="F",
)

Yc = np.asarray(y).reshape(
    (NX, NY),
    order="F",
)

top_band = Yc > 3.15
right_band = Xc > 4.3
bottom_band = Yc < 0.8
left_band = Xc < 0.8

jet_mask = np.asarray(jet_region).reshape(
    (NX, NY),
    order="F",
)

# =====================================================
# PHYSICS
# =====================================================

eq = TransientTerm() == DiffusionTerm(coeff=ALPHA)

# =====================================================
# AIR MIXING
# =====================================================


def apply_air_mixing(
    field,
    cooler_on,
):
    mixed = field.copy()

    # Ceiling flow

    right_flow = np.roll(
        field,
        shift=1,
        axis=0,
    )

    mixed[top_band] = (1 - ROOM_MIXING) * mixed[top_band] + ROOM_MIXING * right_flow[
        top_band
    ]

    # Right wall

    down_flow = np.roll(
        field,
        shift=-1,
        axis=1,
    )

    mixed[right_band] = (1 - ROOM_MIXING) * mixed[right_band] + ROOM_MIXING * down_flow[
        right_band
    ]

    # Floor return

    left_flow = np.roll(
        field,
        shift=-1,
        axis=0,
    )

    mixed[bottom_band] = (1 - ROOM_MIXING) * mixed[
        bottom_band
    ] + ROOM_MIXING * left_flow[bottom_band]

    # Left wall

    up_flow = np.roll(
        field,
        shift=1,
        axis=1,
    )

    mixed[left_band] = (1 - ROOM_MIXING) * mixed[left_band] + ROOM_MIXING * up_flow[
        left_band
    ]

    if cooler_on:
        jet_flow = np.roll(
            field,
            shift=(1, -1),
            axis=(0, 1),
        )

        mixed[jet_mask] = (1 - JET_MIXING) * mixed[jet_mask] + JET_MIXING * jet_flow[
            jet_mask
        ]

    return mixed


# =====================================================
# SNAPSHOTS
# =====================================================

snapshots = []

# =====================================================
# SIMULATION
# =====================================================

for step in tqdm(
    range(STEPS),
    desc="Simulation",
    unit="step",
):
    cooler_on = step >= COOLER_START_STEP

    eq.solve(
        var=T,
        dt=DT,
    )

    # Person

    T.setValue(
        T.value + 0.04 * (PERSON_TEMP - T.value),
        where=person_region,
    )

    # Cooler

    if cooler_on:
        T.setValue(
            T.value + 0.25 * (COOLER_TEMP - T.value),
            where=cooler_region,
        )

        T.setValue(
            T.value + 0.05 * (COOLER_TEMP - T.value),
            where=jet_region,
        )

    # Air movement

    field = T.value.reshape(
        (NX, NY),
        order="F",
    )

    field = apply_air_mixing(
        field,
        cooler_on,
    )

    T.setValue(field.flatten(order="F"))

    # Safety clamp

    T.setValue(
        np.clip(
            T.value,
            15.0,
            55.0,
        )
    )

    if step % SNAPSHOT_EVERY == 0:
        snapshots.append(
            {
                "step": step,
                "cooler": cooler_on,
                "field": field.copy(),
            }
        )

print(f"Stored {len(snapshots)} snapshots")

# =====================================================
# VIDEO
# =====================================================

print("Generating video...")

fig, ax = plt.subplots(figsize=(10, 7))

frame0 = snapshots[0]["field"]

im = ax.imshow(
    frame0.T,
    origin="lower",
    cmap="inferno",
    vmin=15,
    vmax=55,
)

plt.colorbar(im, ax=ax, label="Temperature (°C)")

writer = FFMpegWriter(
    fps=30,
    bitrate=5000,
)

with writer.saving(
    fig,
    VIDEO_FILE,
    dpi=150,
):
    for frame in tqdm(
        snapshots,
        desc="Encoding",
        unit="frame",
    ):
        field = frame["field"]

        im.set_data(field.T)

        ax.clear()

        im = ax.imshow(
            field.T,
            origin="lower",
            cmap="inferno",
            vmin=15,
            vmax=55,
        )

        ax.contour(
            field.T,
            levels=[
                20,
                25,
                30,
                35,
                40,
                45,
                50,
            ],
            colors="white",
            linewidths=0.5,
        )

        cooler_status = "ON" if frame["cooler"] else "OFF"

        ax.set_title(f"Step {frame['step']} | Cooler: {cooler_status}")

        writer.grab_frame()

plt.close(fig)

print()
print("Video written:")
print(VIDEO_FILE)
