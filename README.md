# Simulations Repository

This repository is a collection of small to medium simulation projects built for learning, experimentation, and discussion. It spans physics, markets, agent-based systems, probability, transport, cellular systems, and interactive browser demos.

The codebase is best understood as a simulation playground rather than a single packaged application. Most folders contain standalone scripts, notebooks, or scene files that explore one idea at a time. That makes the repository useful for learners who want examples they can run, modify, and extend.

## Who This Is For

- Learners who want concrete simulation examples across different domains
- Mentees looking for project ideas to study, critique, or build on
- Educators who want lightweight reference material for discussion or demos
- Developers interested in experimenting with numerical models, agents, visualization, or interactive simulations

## What You Will Find

The repository is organized by topic rather than framework:

| Folder | Focus |
| --- | --- |
| `agent-based-modeling/` | NetLogo-style agent-based models such as fire spread, segregation, and market behavior |
| `agentic-gametheory/` | LLM-agent experiments around game theory |
| `cell-simulation/` | Simple biological or growth-oriented simulations |
| `cheminformatics/` | Chemistry-oriented data and visualization experiments |
| `discrete-event/` | Queueing and event-driven systems such as refueling simulations |
| `gameoflife/` | Cellular automata and social simulation experiments |
| `general_mathematics/` | Mathematical visualizations and projection-related code |
| `javascript-simulations/` | Browser-based interactive simulations such as particle systems and fractal trees |
| `market_simulation/` | Market dynamics, repayment models, and agent-based market experiments |
| `monte-carlo-simulations/` | Stochastic simulations, including a wealth/lifetime outcome model |
| `network_simulations/` | Network behavior and adaptation experiments |
| `particle_simulations/` | Particle systems, including a GPU/OpenGL-oriented subproject |
| `physics_simulations_general/` | General mechanics and MuJoCo-based simulations |
| `probabilistic/` | Probability and distribution-focused scripts in MATLAB/Octave style |
| `scientific-computation/` | Numerical and visualization experiments |
| `transport_simulations/` | Transport and multi-agent movement simulations |
| `visualizations_manim/` | Manim-based mathematical or physical visualizations |

## Suggested Starting Points

If you are new to the repository, these are good first projects to open:

- `monte-carlo-simulations/wealth_monte_carlo.py` for a readable stochastic simulation with generated outputs
- `javascript-simulations/bouncing-particles.html` for a quick browser-based interactive example
- `physics_simulations_general/mujoco/box_rain.py` for physics simulation with MuJoCo
- `market_simulation/marketsim.py` for a simple economic system model
- `gameoflife/game.py` for classic rule-based emergent behavior

## Setup

The repository is mainly Python-based, with some JavaScript, C/C++, notebook, and MATLAB-style files. A single environment will not perfectly cover every folder, but this is a practical starting point for the Python projects:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Notes:

- Some subprojects have their own local requirements or external dependencies
- MuJoCo-based examples require MuJoCo to be installed and working on your machine
- GPU/OpenGL examples under `particle_simulations/gpu_particle_sim/` require a native toolchain and graphics libraries
- Notebook-based work is easiest to explore in Jupyter

## Running Simulations

Most projects are run directly from their folder. Examples:

```bash
python monte-carlo-simulations/wealth_monte_carlo.py
python physics_simulations_general/mujoco/box_rain.py
python gameoflife/game.py
python market_simulation/cc_repayment.py
```

For browser-based demos, open the HTML file in a browser:

```bash
xdg-open javascript-simulations/bouncing-particles.html
```

## How To Use This Repository As a Learner

A useful pattern is:

1. Pick one folder that matches your interests
2. Run the simplest script in that folder
3. Change one parameter and observe the effect
4. Write down what the model assumes and what it ignores
5. Extend it with one new rule, visualization, or constraint

This repository is especially useful when treated as a set of study cases rather than as finished products.

## Expectations and Limitations

- Code quality and maturity vary across folders
- Some projects are exploratory or work-in-progress
- Interfaces and conventions are not fully standardized across the repository
- Not every simulation has a dedicated README or complete documentation yet

That said, the collection is valuable precisely because it shows a wide range of modeling styles and implementation approaches in one place.

## For Mentors and Collaborators

This repository can support:

- simulation code reviews
- discussion around modeling assumptions
- project-based learning
- extensions into research prototypes or course assignments

Good mentoring exercises include asking learners to:

- explain the state variables and update rules in a script
- identify unrealistic assumptions
- compare deterministic and stochastic approaches
- add instrumentation, plots, or validation checks
- refactor one standalone script into reusable components

## Repository Status

This is an active learning repository and will continue to evolve as new simulation ideas are added.
