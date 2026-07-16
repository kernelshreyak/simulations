from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass(frozen=True)
class WarpConfig:
    warp_size: int = 32


class MemoryBackend(ABC):
    """Base interface for memory backends."""

    @abstractmethod
    def read(self, addr: int) -> float:
        raise NotImplementedError

    @abstractmethod
    def write(self, addr: int, value: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def bulk_read(self, addrs: list[int]) -> list[float]:
        raise NotImplementedError

    @abstractmethod
    def bulk_write(self, addrs: list[int], values: list[float]) -> None:
        raise NotImplementedError


@dataclass
class LatencyEntry:
    event: str
    latency_cycles: int
    timestamp: int = 0


@dataclass
class MemoryEvent:
    addr: int
    value: float
    timestamp: int
    size_bytes: int = 4


@dataclass
class CacheLine:
    tag: int = -1
    value: float = 0.0
    valid: bool = False
    dirty: bool = False
    touch_timestamp: int = 0


class L1Cache:
    """Small set-associative cache model used by the simulator."""

    def __init__(self, num_lines: int = 64, line_size_words: int = 16):
        if num_lines <= 0:
            raise ValueError("num_lines must be positive")
        if line_size_words <= 0:
            raise ValueError("line_size_words must be positive")
        self.num_lines = num_lines
        self.line_size_bytes = line_size_words * 4
        lines_per_set = 2
        num_sets = max(1, num_lines // lines_per_set)
        self.sets: list[list[CacheLine]] = [
            [CacheLine() for _ in range(lines_per_set)] for _ in range(num_sets)
        ]
        self.accesses = 0
        self.hits = 0
        self._timestamp = 0

    def _get_tag(self, addr: int) -> int:
        return addr // self.line_size_bytes

    def _get_set_index(self, addr: int) -> int:
        return self._get_tag(addr) % len(self.sets)

    def _touch(self, line: CacheLine) -> None:
        self._timestamp += 1
        line.touch_timestamp = self._timestamp

    def _select_victim(self, set_idx: int) -> CacheLine:
        for line in self.sets[set_idx]:
            if not line.valid:
                return line
        return min(self.sets[set_idx], key=lambda line: line.touch_timestamp)

    def read(self, addr: int) -> float:
        self.accesses += 1
        tag = self._get_tag(addr)
        set_idx = self._get_set_index(addr)
        for line in self.sets[set_idx]:
            if line.valid and line.tag == tag:
                self.hits += 1
                self._touch(line)
                return line.value

        victim = self._select_victim(set_idx)
        victim.tag = tag
        victim.value = 0.0
        victim.valid = True
        victim.dirty = False
        self._touch(victim)
        return victim.value

    def write(self, addr: int, value: float) -> None:
        self.accesses += 1
        tag = self._get_tag(addr)
        set_idx = self._get_set_index(addr)
        for line in self.sets[set_idx]:
            if line.valid and line.tag == tag:
                line.value = value
                line.dirty = True
                self._touch(line)
                return

        victim = self._select_victim(set_idx)
        victim.tag = tag
        victim.value = value
        victim.valid = True
        victim.dirty = True
        self._touch(victim)

    def bulk_read(self, addrs: list[int]) -> list[float]:
        return [self.read(addr) for addr in addrs]

    def bulk_write(self, addrs: list[int], values: list[float]) -> None:
        for addr, value in zip(addrs, values):
            self.write(addr, value)

    @property
    def hit_rate(self) -> float:
        if self.accesses == 0:
            return 0.0
        return self.hits / self.accesses


class GlobalMemoryBackend(MemoryBackend):
    """Simulates GPU global memory (HBM/DRAM model)."""

    BASE_LATENCY_CYCLES = 200
    BURST_SPEED_GB_PER_S = 900.0

    def __init__(self, capacity_bytes: int = 1 << 30):
        if capacity_bytes <= 0:
            raise ValueError("capacity_bytes must be positive")
        self.capacity_bytes = capacity_bytes
        self.data: dict[int, float] = {}
        self.latency_cycles = self.BASE_LATENCY_CYCLES
        self.events: list[MemoryEvent] = []
        self.total_bytes_accessed = 0

    def _validate_addr(self, addr: int) -> None:
        if addr < 0:
            raise ValueError("Address must be non-negative")
        if addr >= self.capacity_bytes:
            raise ValueError(
                f"Address {addr} exceeds memory capacity {self.capacity_bytes}"
            )

    def read(self, addr: int) -> float:
        self._validate_addr(addr)
        val = self.data.get(addr // 4, 0.0)
        self.events.append(
            MemoryEvent(addr=addr, value=val, timestamp=self.latency_cycles, size_bytes=4)
        )
        self.total_bytes_accessed += 4
        return val

    def write(self, addr: int, value: float) -> None:
        self._validate_addr(addr)
        word_addr = addr // 4
        self.data[word_addr] = value
        self.total_bytes_accessed += 4
        self.events.append(
            MemoryEvent(addr=addr, value=value, timestamp=self.latency_cycles, size_bytes=4)
        )

    def bulk_read(self, addrs: list[int]) -> list[float]:
        if not addrs:
            return []
        unique_segments = {addr // 256 for addr in addrs}
        self.latency_cycles = self.BASE_LATENCY_CYCLES + max(0, len(unique_segments) - 1) * 4
        return [self.read(addr) for addr in addrs]

    def bulk_write(self, addrs: list[int], values: list[float]) -> None:
        if not addrs:
            return
        unique_segments = {addr // 256 for addr in addrs}
        self.latency_cycles = self.BASE_LATENCY_CYCLES + max(0, len(unique_segments) - 1) * 4
        for addr, val in zip(addrs, values):
            self.write(addr, val)


class SharedMemoryBackend(MemoryBackend):
    """Simulates SM shared memory with bank conflict tracking."""

    BASE_LATENCY_CYCLES = 1
    NUM_BANKS = 32

    def __init__(self, size_words: int = 65536):
        if size_words <= 0:
            raise ValueError("size_words must be positive")
        self.size_words = size_words
        self.banks: list[dict[int, float]] = [{} for _ in range(self.NUM_BANKS)]
        self.bank_conflicts = 0
        self.total_accesses = 0

    def _get_bank(self, word_addr: int) -> int:
        return word_addr % self.NUM_BANKS

    def _validate_addr(self, addr: int) -> int:
        if addr < 0:
            raise ValueError("Address must be non-negative")
        word_addr = addr // 4
        if word_addr >= self.size_words:
            raise ValueError(f"Address {addr} exceeds shared memory size")
        return word_addr

    def read(self, addr: int) -> float:
        self.total_accesses += 1
        word_addr = self._validate_addr(addr)
        bank_idx = self._get_bank(word_addr)
        return self.banks[bank_idx].get(word_addr, 0.0)

    def write(self, addr: int, value: float) -> None:
        self.total_accesses += 1
        word_addr = self._validate_addr(addr)
        bank_idx = self._get_bank(word_addr)
        self.banks[bank_idx][word_addr] = value

    def bulk_read(self, addrs: list[int]) -> list[float]:
        if not addrs:
            return []
        bank_access_counts: dict[int, int] = {}
        for addr in addrs:
            word_addr = self._validate_addr(addr)
            bank_idx = self._get_bank(word_addr)
            bank_access_counts[bank_idx] = bank_access_counts.get(bank_idx, 0) + 1

        self.bank_conflicts += sum(max(0, count - 1) for count in bank_access_counts.values())
        return [self.read(addr) for addr in addrs]

    def bulk_write(self, addrs: list[int], values: list[float]) -> None:
        if not addrs:
            return
        bank_access_counts: dict[int, int] = {}
        for addr in addrs:
            word_addr = self._validate_addr(addr)
            bank_idx = self._get_bank(word_addr)
            bank_access_counts[bank_idx] = bank_access_counts.get(bank_idx, 0) + 1

        self.bank_conflicts += sum(max(0, count - 1) for count in bank_access_counts.values())
        for addr, val in zip(addrs, values):
            self.write(addr, val)

    @property
    def utilization(self) -> float:
        used_banks = sum(1 for bank in self.banks if bank)
        return used_banks / self.NUM_BANKS


class ConstantMemoryBackend(MemoryBackend):
    """Constant memory with broadcast capability."""

    BASE_LATENCY_CYCLES = 4
    MAX_ENTRIES = 65536

    def __init__(self):
        self.data: list[float] = [0.0] * self.MAX_ENTRIES
        self.broadcast_count = 0

    def _index(self, addr: int) -> int:
        if addr < 0:
            raise ValueError("Address must be non-negative")
        return (addr // 4) % self.MAX_ENTRIES

    def read(self, addr: int) -> float:
        return self.data[self._index(addr)]

    def write(self, addr: int, value: float) -> None:
        self.data[self._index(addr)] = value

    def bulk_read(self, addrs: list[int]) -> list[float]:
        if not addrs:
            return []
        if len(set(addrs)) == 1:
            self.broadcast_count += 1
        return [self.read(addr) for addr in addrs]

    def bulk_write(self, addrs: list[int], values: list[float]) -> None:
        for addr, value in zip(addrs, values):
            self.write(addr, value)
