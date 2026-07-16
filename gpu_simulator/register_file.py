"""Register file for simulating GPU thread-local storage."""

from dataclasses import dataclass, field


@dataclass
class Register:
    value: float = 0.0
    is_valid: bool = True


@dataclass
class RegisterFile:
    num_regs: int = 32
    registers: list[Register] = field(default_factory=list)

    def __post_init__(self):
        if self.num_regs <= 0:
            raise ValueError("num_regs must be positive")
        self.registers = [Register() for _ in range(self.num_regs)]

    def read(self, reg_id: int) -> float:
        if not (0 <= reg_id < self.num_regs):
            raise ValueError(f"Invalid register ID: {reg_id}")
        if not self.registers[reg_id].is_valid:
            raise ValueError(f"Register {reg_id} is invalid")
        return self.registers[reg_id].value

    def write(self, reg_id: int, value: float) -> None:
        if not (0 <= reg_id < self.num_regs):
            raise ValueError(f"Invalid register ID: {reg_id}")
        self.registers[reg_id].value = value
        self.registers[reg_id].is_valid = True

    def write_vector(self, start_reg: int, values: list[float]) -> None:
        """Write a vector to consecutive registers."""
        for i, val in enumerate(values):
            self.write(start_reg + i, val)

    def read_vector(self, start_reg: int, count: int) -> list[float]:
        """Read from consecutive registers."""
        return [self.read(start_reg + i) for i in range(count)]

    def zero_all(self) -> None:
        """Zero out all registers and mark valid."""
        for reg in self.registers:
            reg.value = 0.0
            reg.is_valid = True

    def clear_invalid(self) -> None:
        """Mark all registers invalid."""
        for reg in self.registers:
            reg.is_valid = False

    def clone(self) -> "RegisterFile":
        new_rf = RegisterFile(len(self.registers))
        for i, reg in enumerate(self.registers):
            new_rf.registers[i].value = reg.value
            new_rf.registers[i].is_valid = reg.is_valid
        return new_rf

    def as_list(self) -> list[float]:
        """Return the raw register values in register order."""
        return [reg.value for reg in self.registers]
