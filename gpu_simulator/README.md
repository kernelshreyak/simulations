# GPU Simulator

`gpu_simulator/` is a standalone CUDA architecture visualizer built for learning.
It is not trying to run real CUDA binaries. Instead, it gives you a controlled
way to study how CUDA ideas fit together:

- grid launch configuration
- CTAs (thread blocks)
- SM residency limits
- warp issue order
- lanes inside a warp
- registers
- global memory
- shared memory
- constant memory
- coalesced global transactions at a coarse teaching level

The simulator loads a small Python kernel file, schedules CTAs onto SMs,
issues warps round-robin, and records a trace that the Pygame GUI plays back.

![GPU Simulator screenshot](https://github.com/kernelshreyak/gpu-simulator/blob/main/screenshot.png)


## Why This Is Useful

This simulator is most useful if you are trying to understand CUDA mentally
before writing or optimizing real CUDA kernels.

It helps answer questions like:

- What does a grid/block launch actually mean?
- How are CTAs placed onto SMs?
- What is a warp and which lanes are active?
- Why does block size change how many warps exist?
- What does shared memory look like at CTA scope?
- Why do global accesses tend to be discussed in terms of warp transactions?
- How do per-thread registers differ from shared memory?

The point is not perfect hardware fidelity. The point is to make CUDA execution
legible enough that you can watch it happen and connect the abstractions.

## Install And Run

Use `uv` so you do not need to permanently add Pygame to the repo environment.

Run the simulator with the bundled sample:

```bash
uv run --with pygame python -m gpu_simulator
```

Run it with a specific kernel file:

```bash
uv run --with pygame python -m gpu_simulator gpu_simulator/example_kernel.py
```

You can also launch the app entry directly:

```bash
uv run --with pygame python gpu_simulator/app.py
```

If you start it with no file, it automatically loads the bundled sample kernel
so the window is immediately useful.

## GUI Controls

- `L`: load a kernel file
- `R`: rerun the current kernel
- `Space`: play or pause warp issue playback
- `N` or `Right`: next warp issue
- `B` or `Left`: previous warp issue
- `,` or `.`: move the focused lane inside the current warp
- `Up` or `Down`: change playback speed

## What The GUI Shows

- `SM Overview`: which CTAs are resident on each SM and which warp is issuing
- `Warp Lanes`: the lanes in the currently issued warp
- `Scheduler + Stats`: the current issue step, cycle, CTA, warp, and memory stats
- `Global Memory`: a compact view of global memory words touched so far
- `Shared Memory`: the current CTA's shared memory state
- `Focused Lane`: one lane's registers, memory operations, notes, and result

This is intentionally organized around CUDA vocabulary so you can map what you
see in the window to CUDA documentation and CUDA optimization discussions.

## How To Learn CUDA With It

A good study loop is:

1. Run the bundled sample and watch one warp at a time.
2. Compare `grid_dim`, `block_dim`, and `warp_size` with the number of CTAs,
   warps, and lanes you see in the GUI.
3. Look at one lane's register writes and compare them with shared/global memory.
4. Change the block size and rerun.
5. Change the number of SMs or `max_blocks_per_sm` and observe CTA residency.
6. Change memory access patterns and watch the transaction count and memory maps.

That sequence gives you a practical feel for occupancy, locality, and
execution granularity without needing to debug on a real GPU first.

## Kernel File Format

A kernel file is a small Python script that defines launch metadata plus a
single `kernel(ctx)` function.

Required:

- `kernel(ctx)`

Optional:

- `initialize(sim)`
- `TITLE`
- `DESCRIPTION`
- `GRID_DIM`
- `BLOCK_DIM`
- `SHARED_MEMORY_BYTES`
- `REGISTERS_PER_THREAD`
- `WARP_SIZE`
- `ARCH`

Example:

```python
TITLE = "My Kernel"
GRID_DIM = (4, 1, 1)
BLOCK_DIM = (16, 1, 1)
WARP_SIZE = 8
SHARED_MEMORY_BYTES = 256

ARCH = {
    "num_sms": 2,
    "max_blocks_per_sm": 2,
}

def initialize(sim):
    sim.global_memory.write(0, 1.0)

def kernel(ctx):
    value = ctx.load_global(ctx.global_linear_idx * 4)
    ctx.write_reg(0, value)
    ctx.store_shared(ctx.linear_thread_idx * 4, value)
    ctx.note("staged one value into shared memory")
    return value
```

## CUDA Concepts Mapped Into The Simulator

- `GRID_DIM`: CUDA grid dimensions
- `BLOCK_DIM`: CUDA CTA dimensions
- `WARP_SIZE`: warp width used by the teaching model
- `ARCH["num_sms"]`: number of SMs available for CTA scheduling
- `ARCH["max_blocks_per_sm"]`: CTA residency limit per SM
- `ctx.block_idx`: block index
- `ctx.thread_idx`: thread index inside the CTA
- `ctx.warp_idx`: warp index inside the CTA
- `ctx.lane_id`: lane inside the issued warp
- `ctx.global_linear_idx`: flattened global thread id for simple kernels

Available context methods:

- `ctx.load_global(addr)`
- `ctx.store_global(addr, value)`
- `ctx.load_shared(addr)`
- `ctx.store_shared(addr, value)`
- `ctx.load_constant(addr)`
- `ctx.write_reg(reg_id, value)`
- `ctx.read_reg(reg_id)`
- `ctx.note(message)`

## Bundled Sample

[example_kernel.py](/home/shreyak/programming/simulations/gpu_simulator/example_kernel.py)
stages a tile from global memory into shared memory, reads a neighboring shared
value, and writes a blended result back to global memory. It is meant to show:

- CTA-local shared memory
- warp issue order
- lane-local registers
- constant memory reads
- the difference between global and shared memory traffic

If you want to understand CUDA better, this bundled sample is the right first
thing to run.
