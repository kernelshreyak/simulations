import os
import shutil
import subprocess

import numpy as np
import taichi as ti

# --------------------------------------------------
# Configuration
# --------------------------------------------------

N = 1000

WIDTH = 1280
HEIGHT = 720

FPS = 60
DT = 1.0 / FPS

TOTAL_FRAMES = 300
SUBSTEPS = 4

RADIUS = 0.04

GRAVITY = ti.Vector([0.0, -9.81, 0.0])

BOUNCE = 0.8
DAMPING = 0.9998

FRAME_DIR = "frames"

# --------------------------------------------------

ti.init(arch=ti.cuda)

os.makedirs(FRAME_DIR, exist_ok=True)

# remove old frames
for f in os.listdir(FRAME_DIR):
    if f.endswith(".png"):
        os.remove(os.path.join(FRAME_DIR, f))

# --------------------------------------------------
# Fields
# --------------------------------------------------

pos = ti.Vector.field(3, dtype=ti.f32, shape=N)
vel = ti.Vector.field(3, dtype=ti.f32, shape=N)

# --------------------------------------------------
# Ground mesh
# --------------------------------------------------

ground_vertices = ti.Vector.field(3, dtype=ti.f32, shape=4)
ground_indices = ti.field(dtype=ti.i32, shape=6)

ground_vertices.from_numpy(
    np.array(
        [
            [-6, 0, -6],
            [6, 0, -6],
            [6, 0, 6],
            [-6, 0, 6],
        ],
        dtype=np.float32,
    )
)

ground_indices.from_numpy(np.array([0, 1, 2, 0, 2, 3], dtype=np.int32))

# --------------------------------------------------
# Initialization
# --------------------------------------------------

xs = (np.arange(N) % 20) * 0.22 - 2.1
zs = ((np.arange(N) // 20) % 20) * 0.22 - 2.1

# small random horizontal offset
xs += np.random.uniform(-0.05, 0.05, N)
zs += np.random.uniform(-0.05, 0.05, N)

# Gaussian height distribution
ys = np.random.normal(loc=4.0, scale=1.5, size=N)

ys = np.clip(ys, 1.0, 8.0)

positions = np.stack([xs, ys, zs], axis=1).astype(np.float32)

velocities = np.zeros((N, 3), dtype=np.float32)

velocities[:, 0] = np.random.uniform(-0.5, 0.5, N)
velocities[:, 2] = np.random.uniform(-0.5, 0.5, N)

pos.from_numpy(positions)
vel.from_numpy(velocities)

# --------------------------------------------------
# Physics
# --------------------------------------------------


@ti.kernel
def step():

    for i in pos:
        vel[i] += GRAVITY * DT
        vel[i] *= DAMPING

        pos[i] += vel[i] * DT

        if pos[i].y < RADIUS:
            pos[i].y = RADIUS

            if vel[i].y < 0:
                vel[i].y *= -BOUNCE

            vel[i].x *= 0.985
            vel[i].z *= 0.985


# --------------------------------------------------
# Renderer
# --------------------------------------------------

window = ti.ui.Window("Taichi", (WIDTH, HEIGHT), show_window=False)

canvas = window.get_canvas()

scene = ti.ui.Scene()

camera = ti.ui.Camera()

camera.position(5.5, 4.5, 7.5)
camera.lookat(0, 2, 0)
camera.fov(45)

print("Rendering...")

for frame in range(TOTAL_FRAMES):
    for _ in range(SUBSTEPS):
        step()

    scene.set_camera(camera)

    scene.ambient_light((0.55, 0.55, 0.55))

    scene.point_light(pos=(4, 8, 4), color=(1, 1, 1))

    scene.mesh(ground_vertices, indices=ground_indices, color=(0.75, 0.75, 0.75))

    scene.particles(pos, radius=RADIUS, color=(0.25, 0.55, 1.0))

    canvas.scene(scene)

    window.save_image(os.path.join(FRAME_DIR, f"frame_{frame:05d}.png"))

    if frame % 30 == 0:
        print(f"{frame}/{TOTAL_FRAMES}")

# --------------------------------------------------
# Encode video
# --------------------------------------------------

print("Encoding video...")

encoders = subprocess.run(
    ["ffmpeg", "-encoders"], capture_output=True, text=True
).stdout

if "h264_nvenc" in encoders:
    codec = ["-c:v", "h264_nvenc", "-preset", "p4", "-cq", "22", "-b:v", "0"]

else:
    codec = ["-c:v", "libx264", "-crf", "20", "-preset", "medium"]

subprocess.run(
    [
        "ffmpeg",
        "-y",
        "-framerate",
        str(FPS),
        "-i",
        os.path.join(FRAME_DIR, "frame_%05d.png"),
        *codec,
        "-pix_fmt",
        "yuv420p",
        "balls.mp4",
    ],
    check=True,
)

print()
print("Done.")
print("Video written to balls.mp4")
