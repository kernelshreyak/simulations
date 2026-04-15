- This is a “simulation codes” repository focused on many topics, mainly in Python and some C/C++ (per `README.md`).
- The root organizes work by domain, with top-level folders like `agent-based-modeling`, `discrete-event`, `monte-carlo-simulations`, and `network_simulations`.
- There’s a strong physics/particle emphasis via `physics_simulations_general` and `particle_simulations`.
- The repo includes classic cellular automata content under `gameoflife` and `cell-simulation`.
- Market and economic dynamics are represented in `market_simulation` and `agentic-gametheory`.
- There’s a JavaScript-focused area for simulations in `javascript-simulations`.
- Visualization work appears isolated in `visualizations_manim`, implying Manim-based rendering.
- Domain-specific simulations include `cheminformatics`, `transport_simulations`, and `instruct-ai-robot`.
- The root includes a standalone utility `myplant.py` (terminal-based plant care tracker per `README.md`).
- The repo tracks dependencies via `requirements.txt` and includes a local `venv` directory for Python environment management.

## Architecture (brief)

- Domain-first layout: top-level folders group simulations by topic (e.g., agent-based, Monte Carlo, physics, transport).
- Polyglot implementations: primarily Python with some C/C++ and JavaScript-based simulations.
- Standalone scripts: most simulations are runnable, self-contained files inside their domain folders.
- Shared environment: `requirements.txt` captures Python dependencies; a local `venv` is present for isolation.
- Visualization paths: dedicated `visualizations_manim` plus simulation folders that emit plots/animations.
