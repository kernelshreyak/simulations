# MuJoCo Simulations

A collection of physics simulations powered by MuJoCo.

## Requirements

- Python 3
- mujoco
- numpy

Install dependencies (if needed):

```bash
pip install mujoco numpy
```

## Run

Examples:

```bash
python box_rain.py
python bouncing_ball.py
python domino_sim.py
python mixer_new.py
python test_mujoco.py
```

## Box Rain

`box_rain.py` generates a randomized set of falling boxes and writes a fresh
`box_rain.xml` before launching the viewer. Current defaults in the script:

- 100 boxes
- z range: 2.0 to 14.0
- xy spread: 1.2
- box size range: 0.05 to 0.30

## Other Files

- `direct/`: standalone XML scenes that can be loaded by MuJoCo tools.
- `mjcf/`: additional MJCF assets.
