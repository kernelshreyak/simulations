TITLE = "Shared Memory Staging Demo"
DESCRIPTION = (
    "A CUDA-style teaching kernel: each CTA stages a tile from global memory into "
    "shared memory, then each lane reads a neighbor from that shared tile and "
    "writes a blended result back to global memory."
)
GRID_DIM = (4, 1, 1)
BLOCK_DIM = (16, 1, 1)
SHARED_MEMORY_BYTES = 16 * 4
REGISTERS_PER_THREAD = 12
WARP_SIZE = 8

ARCH = {
    "num_sms": 2,
    "max_blocks_per_sm": 2,
    "max_threads_per_sm": 1024,
    "max_warps_per_sm": 32,
    "shared_memory_per_sm_bytes": 64 * 1024,
    "registers_per_sm": 65536,
}


def initialize(sim):
    total_threads = GRID_DIM[0] * BLOCK_DIM[0]
    input_base = 0
    weights_base = 4096
    bias_base = 8192
    for idx in range(total_threads):
        sim.global_memory.write(input_base + idx * 4, float(idx + 1))
        sim.global_memory.write(weights_base + idx * 4, float((idx % 8) + 1))
    sim.constant_memory.write(bias_base, 3.0)


def kernel(ctx):
    input_base = 0
    weights_base = 4096
    output_base = 12288
    bias_base = 8192

    global_idx = ctx.global_linear_idx
    lane = ctx.lane_id
    local_idx = ctx.linear_thread_idx
    tile_addr = local_idx * 4

    value = ctx.load_global(input_base + global_idx * 4)
    weight = ctx.load_global(weights_base + global_idx * 4)
    bias = ctx.load_constant(bias_base)

    ctx.write_reg(0, value)
    ctx.write_reg(1, weight)
    ctx.write_reg(2, bias)

    ctx.store_shared(tile_addr, value * weight)
    neighbor_addr = ((local_idx + 1) % ctx.block_dim[0]) * 4
    neighbor = ctx.load_shared(neighbor_addr)
    result = value * weight + neighbor + bias

    ctx.write_reg(3, neighbor)
    ctx.write_reg(4, result)
    ctx.store_global(output_base + global_idx * 4, result)
    ctx.note(
        f"SM{ctx.sm_id} CTA{ctx.block_idx[0]} warp{ctx.warp_idx} lane{lane} "
        f"used shared[{local_idx}] and shared[{(local_idx + 1) % ctx.block_dim[0]}]"
    )
    return result
