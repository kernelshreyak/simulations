from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import ceil
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable, Iterable

try:
    from .memory_backend import ConstantMemoryBackend, GlobalMemoryBackend, SharedMemoryBackend
    from .register_file import RegisterFile
except ImportError:  # pragma: no cover
    from memory_backend import ConstantMemoryBackend, GlobalMemoryBackend, SharedMemoryBackend
    from register_file import RegisterFile


Dim3 = tuple[int, int, int]


def _normalize_dim(value: int | Iterable[int]) -> Dim3:
    if isinstance(value, int):
        if value <= 0:
            raise ValueError("Dimensions must be positive")
        return (value, 1, 1)
    parts = tuple(int(part) for part in value)
    if len(parts) == 1:
        parts = (parts[0], 1, 1)
    elif len(parts) == 2:
        parts = (parts[0], parts[1], 1)
    elif len(parts) != 3:
        raise ValueError("Dimensions must have 1, 2, or 3 elements")
    if any(part <= 0 for part in parts):
        raise ValueError("Dimensions must be positive")
    return parts  # type: ignore[return-value]


def _product(dim: Dim3) -> int:
    return dim[0] * dim[1] * dim[2]


def _flatten_shared_memory(shared_memory: SharedMemoryBackend) -> dict[int, float]:
    flat: dict[int, float] = {}
    for bank in shared_memory.banks:
        flat.update(bank)
    return dict(sorted(flat.items()))


@dataclass(frozen=True)
class ArchitectureConfig:
    num_sms: int = 4
    max_blocks_per_sm: int = 2
    max_threads_per_sm: int = 1024
    max_warps_per_sm: int = 32
    shared_memory_per_sm_bytes: int = 64 * 1024
    registers_per_sm: int = 65536


@dataclass(frozen=True)
class KernelLaunchConfig:
    grid_dim: Dim3
    block_dim: Dim3
    shared_memory_bytes: int = 48 * 1024
    registers_per_thread: int = 32
    warp_size: int = 32

    @property
    def num_blocks(self) -> int:
        return _product(self.grid_dim)

    @property
    def threads_per_block(self) -> int:
        return _product(self.block_dim)

    @property
    def total_threads(self) -> int:
        return self.num_blocks * self.threads_per_block

    @property
    def warps_per_block(self) -> int:
        return ceil(self.threads_per_block / self.warp_size)


@dataclass
class SimulationStats:
    blocks_executed: int = 0
    threads_executed: int = 0
    warps_executed: int = 0
    global_bytes_accessed: int = 0
    global_memory_transactions: int = 0
    shared_memory_accesses: int = 0
    shared_bank_conflicts: int = 0
    constant_broadcasts: int = 0
    waves_launched: int = 0

    def as_dict(self) -> dict[str, int]:
        return {
            "blocks_executed": self.blocks_executed,
            "threads_executed": self.threads_executed,
            "warps_executed": self.warps_executed,
            "global_bytes_accessed": self.global_bytes_accessed,
            "global_memory_transactions": self.global_memory_transactions,
            "shared_memory_accesses": self.shared_memory_accesses,
            "shared_bank_conflicts": self.shared_bank_conflicts,
            "constant_broadcasts": self.constant_broadcasts,
            "waves_launched": self.waves_launched,
        }


@dataclass(frozen=True)
class MemoryOperation:
    memory_space: str
    op: str
    addr: int
    value: float


@dataclass
class LaneTrace:
    thread_idx: Dim3
    lane_id: int
    global_linear_idx: int
    registers: list[float]
    memory_ops: list[MemoryOperation]
    notes: list[str]
    result: Any


@dataclass(frozen=True)
class BlockSnapshot:
    block_idx: Dim3
    sm_id: int
    next_warp: int
    total_warps: int
    complete: bool


@dataclass(frozen=True)
class SMSnapshot:
    sm_id: int
    resident_blocks: list[BlockSnapshot]
    active_block: Dim3 | None
    active_warp: int | None


@dataclass
class WarpTrace:
    step: int
    cycle: int
    sm_id: int
    block_idx: Dim3
    warp_id: int
    active_lanes: list[int]
    lane_traces: list[LaneTrace]
    global_memory_words: dict[int, float]
    shared_memory_words: dict[int, float]
    sm_snapshots: list[SMSnapshot]
    stats: dict[str, int]
    scheduler_note: str


@dataclass
class LaunchResult:
    config: KernelLaunchConfig
    architecture: ArchitectureConfig
    stats: SimulationStats
    traces: list[WarpTrace] = field(default_factory=list)
    thread_results: dict[tuple[Dim3, Dim3], Any] = field(default_factory=dict)
    title: str = "GPU Simulator"
    description: str = ""
    source_path: str = ""


@dataclass
class KernelSpec:
    title: str
    description: str
    grid_dim: Dim3
    block_dim: Dim3
    shared_memory_bytes: int
    kernel: Callable[["ThreadContext"], Any]
    initialize: Callable[["GPUSimulator"], None] | None = None
    registers_per_thread: int = 32
    warp_size: int = 32
    architecture: ArchitectureConfig = field(default_factory=ArchitectureConfig)
    source_path: str = ""


@dataclass
class ThreadContext:
    global_memory: GlobalMemoryBackend
    shared_memory: SharedMemoryBackend
    constant_memory: ConstantMemoryBackend
    registers: RegisterFile
    grid_dim: Dim3
    block_dim: Dim3
    block_idx: Dim3
    thread_idx: Dim3
    warp_idx: int
    lane_id: int
    sm_id: int
    memory_ops: list[MemoryOperation] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def read_reg(self, reg_id: int) -> float:
        value = self.registers.read(reg_id)
        self.notes.append(f"read r{reg_id} -> {value:.3f}")
        return value

    def write_reg(self, reg_id: int, value: float) -> None:
        self.registers.write(reg_id, value)
        self.notes.append(f"write r{reg_id} <- {value:.3f}")

    def load_global(self, addr: int) -> float:
        value = self.global_memory.read(addr)
        self.memory_ops.append(MemoryOperation("global", "read", addr, value))
        return value

    def store_global(self, addr: int, value: float) -> None:
        self.global_memory.write(addr, value)
        self.memory_ops.append(MemoryOperation("global", "write", addr, value))

    def load_shared(self, addr: int) -> float:
        value = self.shared_memory.read(addr)
        self.memory_ops.append(MemoryOperation("shared", "read", addr, value))
        return value

    def store_shared(self, addr: int, value: float) -> None:
        self.shared_memory.write(addr, value)
        self.memory_ops.append(MemoryOperation("shared", "write", addr, value))

    def load_constant(self, addr: int) -> float:
        value = self.constant_memory.read(addr)
        self.memory_ops.append(MemoryOperation("constant", "read", addr, value))
        return value

    def note(self, message: str) -> None:
        self.notes.append(message)

    @property
    def linear_thread_idx(self) -> int:
        tx, ty, tz = self.thread_idx
        bx, by, _ = self.block_dim
        return tx + ty * bx + tz * bx * by

    @property
    def global_linear_idx(self) -> int:
        gx, gy, _ = self.grid_dim
        bx, by, bz = self.block_idx
        block_linear_idx = bx + by * gx + bz * gx * gy
        return block_linear_idx * _product(self.block_dim) + self.linear_thread_idx


@dataclass
class BlockState:
    block_idx: Dim3
    sm_id: int
    shared_memory: SharedMemoryBackend
    total_warps: int
    next_warp: int = 0

    @property
    def complete(self) -> bool:
        return self.next_warp >= self.total_warps


class GPUSimulator:
    """CUDA-oriented teaching simulator with SM and warp scheduling."""

    def __init__(
        self,
        *,
        warp_size: int = 32,
        registers_per_thread: int = 32,
        global_memory_capacity: int = 1 << 20,
    ):
        self.warp_size = warp_size
        self.registers_per_thread = registers_per_thread
        self.global_memory_capacity = global_memory_capacity
        self.global_memory = GlobalMemoryBackend(capacity_bytes=global_memory_capacity)
        self.constant_memory = ConstantMemoryBackend()
        self.last_launch: LaunchResult | None = None

    def reset(self) -> None:
        self.global_memory = GlobalMemoryBackend(capacity_bytes=self.global_memory_capacity)
        self.constant_memory = ConstantMemoryBackend()
        self.last_launch = None

    def load_kernel_spec(self, path: str | Path) -> KernelSpec:
        source_path = str(Path(path).resolve())
        namespace: dict[str, Any] = {
            "__builtins__": __builtins__,
            "MappingProxyType": MappingProxyType,
        }
        code = Path(source_path).read_text(encoding="utf-8")
        exec(compile(code, source_path, "exec"), namespace)

        kernel = namespace.get("kernel")
        if not callable(kernel):
            raise ValueError("Kernel file must define a callable kernel(ctx)")

        initialize = namespace.get("initialize")
        if initialize is not None and not callable(initialize):
            raise ValueError("initialize must be callable when provided")

        architecture_dict = namespace.get("ARCH", {})
        architecture = ArchitectureConfig(
            num_sms=int(architecture_dict.get("num_sms", 4)),
            max_blocks_per_sm=int(architecture_dict.get("max_blocks_per_sm", 2)),
            max_threads_per_sm=int(architecture_dict.get("max_threads_per_sm", 1024)),
            max_warps_per_sm=int(architecture_dict.get("max_warps_per_sm", 32)),
            shared_memory_per_sm_bytes=int(
                architecture_dict.get("shared_memory_per_sm_bytes", 64 * 1024)
            ),
            registers_per_sm=int(architecture_dict.get("registers_per_sm", 65536)),
        )

        return KernelSpec(
            title=str(namespace.get("TITLE", Path(source_path).stem)),
            description=str(namespace.get("DESCRIPTION", "")),
            grid_dim=_normalize_dim(namespace.get("GRID_DIM", (1, 1, 1))),
            block_dim=_normalize_dim(namespace.get("BLOCK_DIM", (32, 1, 1))),
            shared_memory_bytes=int(namespace.get("SHARED_MEMORY_BYTES", 256)),
            kernel=kernel,
            initialize=initialize,
            registers_per_thread=int(namespace.get("REGISTERS_PER_THREAD", 32)),
            warp_size=int(namespace.get("WARP_SIZE", 32)),
            architecture=architecture,
            source_path=source_path,
        )

    def _all_block_indices(self, config: KernelLaunchConfig) -> list[Dim3]:
        gx, gy, gz = config.grid_dim
        return [
            (bx, by, bz)
            for bz in range(gz)
            for by in range(gy)
            for bx in range(gx)
        ]

    def _make_sm_snapshots(
        self,
        resident_blocks: list[list[BlockState]],
        active_sm_id: int,
        active_block_idx: Dim3,
        active_warp: int,
    ) -> list[SMSnapshot]:
        snapshots: list[SMSnapshot] = []
        for sm_id, blocks in enumerate(resident_blocks):
            snapshots.append(
                SMSnapshot(
                    sm_id=sm_id,
                    resident_blocks=[
                        BlockSnapshot(
                            block_idx=block.block_idx,
                            sm_id=sm_id,
                            next_warp=block.next_warp,
                            total_warps=block.total_warps,
                            complete=block.complete,
                        )
                        for block in blocks
                    ],
                    active_block=active_block_idx if sm_id == active_sm_id else None,
                    active_warp=active_warp if sm_id == active_sm_id else None,
                )
            )
        return snapshots

    def _lane_thread_idx(self, lane_linear: int, block_dim: Dim3) -> Dim3:
        bx, by, _ = block_dim
        tz, rem = divmod(lane_linear, bx * by)
        ty, tx = divmod(rem, bx)
        return (tx, ty, tz)

    def _count_global_transactions(self, lane_traces: list[LaneTrace]) -> int:
        segments: set[tuple[str, int]] = set()
        for lane in lane_traces:
            for op in lane.memory_ops:
                if op.memory_space == "global":
                    segments.add((op.op, op.addr // 128))
        return len(segments)

    def run_spec(self, spec: KernelSpec) -> LaunchResult:
        self.warp_size = spec.warp_size
        self.registers_per_thread = spec.registers_per_thread
        self.reset()

        if spec.initialize is not None:
            spec.initialize(self)

        config = KernelLaunchConfig(
            grid_dim=spec.grid_dim,
            block_dim=spec.block_dim,
            shared_memory_bytes=spec.shared_memory_bytes,
            registers_per_thread=spec.registers_per_thread,
            warp_size=spec.warp_size,
        )
        architecture = spec.architecture
        if config.threads_per_block > architecture.max_threads_per_sm:
            raise ValueError("threads per block exceed max_threads_per_sm")

        stats = SimulationStats()
        traces: list[WarpTrace] = []
        thread_results: dict[tuple[Dim3, Dim3], Any] = {}
        start_global_bytes = self.global_memory.total_bytes_accessed
        start_constant_broadcasts = self.constant_memory.broadcast_count
        resident_blocks: list[list[BlockState]] = [[] for _ in range(architecture.num_sms)]
        pending_blocks: deque[Dim3] = deque(self._all_block_indices(config))
        sm_rr_cursor = [0 for _ in range(architecture.num_sms)]
        cycle = 0
        step = 0
        cumulative_shared_accesses = 0
        cumulative_shared_conflicts = 0

        def can_reside(blocks: list[BlockState]) -> bool:
            if len(blocks) >= architecture.max_blocks_per_sm:
                return False
            total_threads = len(blocks) * config.threads_per_block
            total_warps = len(blocks) * config.warps_per_block
            total_shared = len(blocks) * config.shared_memory_bytes
            total_registers = (
                len(blocks) * config.threads_per_block * config.registers_per_thread
            )
            return (
                total_threads + config.threads_per_block <= architecture.max_threads_per_sm
                and total_warps + config.warps_per_block <= architecture.max_warps_per_sm
                and total_shared + config.shared_memory_bytes
                <= architecture.shared_memory_per_sm_bytes
                and total_registers + (
                    config.threads_per_block * config.registers_per_thread
                )
                <= architecture.registers_per_sm
            )

        while True:
            admitted_this_wave = False
            for sm_id in range(architecture.num_sms):
                while pending_blocks and can_reside(resident_blocks[sm_id]):
                    block_idx = pending_blocks.popleft()
                    resident_blocks[sm_id].append(
                        BlockState(
                            block_idx=block_idx,
                            sm_id=sm_id,
                            shared_memory=SharedMemoryBackend(
                                size_words=max(1, config.shared_memory_bytes // 4)
                            ),
                            total_warps=config.warps_per_block,
                        )
                    )
                    stats.blocks_executed += 1
                    admitted_this_wave = True
            if admitted_this_wave:
                stats.waves_launched += 1

            if not pending_blocks and not any(resident_blocks):
                break

            progress_made = False
            for sm_id, blocks in enumerate(resident_blocks):
                if not blocks:
                    continue
                progress_made = True
                cycle += 1
                start_index = sm_rr_cursor[sm_id] % len(blocks)
                block = blocks[start_index]
                sm_rr_cursor[sm_id] += 1

                warp_id = block.next_warp
                lane_start = warp_id * config.warp_size
                lane_end = min(lane_start + config.warp_size, config.threads_per_block)
                active_lanes = list(range(lane_end - lane_start))
                lane_traces: list[LaneTrace] = []

                for lane_linear in range(lane_start, lane_end):
                    lane_id = lane_linear - lane_start
                    thread_idx = self._lane_thread_idx(lane_linear, config.block_dim)
                    context = ThreadContext(
                        global_memory=self.global_memory,
                        shared_memory=block.shared_memory,
                        constant_memory=self.constant_memory,
                        registers=RegisterFile(config.registers_per_thread),
                        grid_dim=config.grid_dim,
                        block_dim=config.block_dim,
                        block_idx=block.block_idx,
                        thread_idx=thread_idx,
                        warp_idx=warp_id,
                        lane_id=lane_id,
                        sm_id=sm_id,
                    )
                    result = spec.kernel(context)
                    stats.threads_executed += 1
                    thread_results[(block.block_idx, thread_idx)] = result
                    lane_traces.append(
                        LaneTrace(
                            thread_idx=thread_idx,
                            lane_id=lane_id,
                            global_linear_idx=context.global_linear_idx,
                            registers=context.registers.as_list(),
                            memory_ops=list(context.memory_ops),
                            notes=list(context.notes),
                            result=result,
                        )
                    )

                block.next_warp += 1
                stats.warps_executed += 1
                stats.global_memory_transactions += self._count_global_transactions(
                    lane_traces
                )
                cumulative_shared_accesses = sum(
                    candidate.shared_memory.total_accesses
                    for sm_blocks in resident_blocks
                    for candidate in sm_blocks
                )
                cumulative_shared_conflicts = sum(
                    candidate.shared_memory.bank_conflicts
                    for sm_blocks in resident_blocks
                    for candidate in sm_blocks
                )
                step += 1

                traces.append(
                    WarpTrace(
                        step=step,
                        cycle=cycle,
                        sm_id=sm_id,
                        block_idx=block.block_idx,
                        warp_id=warp_id,
                        active_lanes=active_lanes,
                        lane_traces=lane_traces,
                        global_memory_words=dict(sorted(self.global_memory.data.items())),
                        shared_memory_words=_flatten_shared_memory(block.shared_memory),
                        sm_snapshots=self._make_sm_snapshots(
                            resident_blocks, sm_id, block.block_idx, warp_id
                        ),
                        stats={
                            "global_bytes_accessed": self.global_memory.total_bytes_accessed
                            - start_global_bytes,
                            "global_transactions": stats.global_memory_transactions,
                            "shared_accesses": cumulative_shared_accesses,
                            "shared_bank_conflicts": cumulative_shared_conflicts,
                        },
                        scheduler_note=(
                            f"SM {sm_id} issued warp {warp_id} from block {block.block_idx}"
                        ),
                    )
                )

                resident_blocks[sm_id] = [candidate for candidate in blocks if not candidate.complete]
                if resident_blocks[sm_id]:
                    sm_rr_cursor[sm_id] %= len(resident_blocks[sm_id])
                else:
                    sm_rr_cursor[sm_id] = 0

            if not progress_made:
                break

        stats.global_bytes_accessed = self.global_memory.total_bytes_accessed - start_global_bytes
        stats.shared_memory_accesses = cumulative_shared_accesses
        stats.shared_bank_conflicts = cumulative_shared_conflicts
        stats.constant_broadcasts = (
            self.constant_memory.broadcast_count - start_constant_broadcasts
        )

        result = LaunchResult(
            config=config,
            architecture=architecture,
            stats=stats,
            traces=traces,
            thread_results=thread_results,
            title=spec.title,
            description=spec.description,
            source_path=spec.source_path,
        )
        self.last_launch = result
        return result

    def run_file(self, path: str | Path) -> LaunchResult:
        return self.run_spec(self.load_kernel_spec(path))


__all__ = [
    "ArchitectureConfig",
    "Dim3",
    "GPUSimulator",
    "KernelLaunchConfig",
    "KernelSpec",
    "LaneTrace",
    "LaunchResult",
    "MemoryOperation",
    "SimulationStats",
    "SMSnapshot",
    "ThreadContext",
    "WarpTrace",
]
