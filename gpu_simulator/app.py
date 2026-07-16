from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

try:
    from .simulator import GPUSimulator, LaunchResult, LaneTrace, WarpTrace
except ImportError:  # pragma: no cover
    from simulator import GPUSimulator, LaunchResult, LaneTrace, WarpTrace

try:
    import pygame
except ImportError:  # pragma: no cover
    pygame = None

try:
    import tkinter as tk
    from tkinter import filedialog
except Exception:  # pragma: no cover
    tk = None
    filedialog = None


WINDOW_WIDTH = 1500
WINDOW_HEIGHT = 960
BACKGROUND = (10, 13, 17)
PANEL = (20, 26, 34)
PANEL_ALT = (28, 36, 46)
TEXT = (234, 238, 244)
MUTED = (150, 161, 176)
ACCENT = (84, 201, 255)
ACTIVE = (110, 221, 155)
HIGHLIGHT = (255, 198, 92)
WARNING = (255, 120, 120)
IDLE = (58, 69, 82)


def default_kernel_path() -> str:
    return str((Path(__file__).resolve().parent / "example_kernel.py").resolve())


def _choose_file() -> str | None:
    if tk is None or filedialog is None:
        return None
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    try:
        file_path = filedialog.askopenfilename(
            title="Select CUDA teaching kernel",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")],
        )
    finally:
        root.destroy()
    return file_path or None


class Button:
    def __init__(self, rect: "pygame.Rect", label: str):
        self.rect = rect
        self.label = label

    def draw(self, surface: "pygame.Surface", font: "pygame.font.Font") -> None:
        pygame.draw.rect(surface, PANEL_ALT, self.rect, border_radius=10)
        pygame.draw.rect(surface, (78, 91, 104), self.rect, width=1, border_radius=10)
        text = font.render(self.label, True, TEXT)
        surface.blit(text, text.get_rect(center=self.rect.center))

    def hit(self, pos: tuple[int, int]) -> bool:
        return self.rect.collidepoint(pos)


class SimulatorGUI:
    def __init__(self, kernel_path: str | None = None):
        if pygame is None:
            raise RuntimeError(
                "pygame is not installed. Use `uv run --with pygame python -m gpu_simulator`."
            )

        pygame.init()
        pygame.display.set_caption("CUDA Architecture Visualizer")
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.clock = pygame.time.Clock()
        self.title_font = pygame.font.SysFont("consolas", 28, bold=True)
        self.body_font = pygame.font.SysFont("consolas", 18)
        self.small_font = pygame.font.SysFont("consolas", 14)
        self.simulator = GPUSimulator()
        self.result: LaunchResult | None = None
        self.kernel_path = kernel_path or default_kernel_path()
        self.current_step = 0
        self.playing = False
        self.playback_fps = 4
        self.focus_lane = 0
        self.message = "Loading bundled sample kernel."
        self.buttons = {
            "load": Button(pygame.Rect(24, 20, 110, 38), "Load [L]"),
            "run": Button(pygame.Rect(144, 20, 110, 38), "Run [R]"),
            "play": Button(pygame.Rect(264, 20, 120, 38), "Play [Space]"),
            "step": Button(pygame.Rect(394, 20, 110, 38), "Step [N]"),
            "back": Button(pygame.Rect(514, 20, 110, 38), "Back [B]"),
            "lane": Button(pygame.Rect(634, 20, 130, 38), "Lane [,/.]"),
        }
        self.load_and_run(self.kernel_path)

    def load_and_run(self, path: str) -> None:
        try:
            self.result = self.simulator.run_file(path)
            self.kernel_path = path
            self.current_step = 0 if self.result.traces else -1
            self.playing = False
            self.focus_lane = 0
            self.message = (
                f"Loaded {Path(path).name}: {self.result.config.num_blocks} CTAs, "
                f"{self.result.architecture.num_sms} SMs, {len(self.result.traces)} warp issues."
            )
        except Exception as exc:  # pragma: no cover
            self.result = None
            self.playing = False
            self.message = f"Load failed: {exc}"

    def current_trace(self) -> WarpTrace | None:
        if self.result is None or not self.result.traces or self.current_step < 0:
            return None
        return self.result.traces[self.current_step]

    def current_lane(self, trace: WarpTrace | None) -> LaneTrace | None:
        if trace is None or not trace.lane_traces:
            return None
        lane_index = min(self.focus_lane, len(trace.lane_traces) - 1)
        return trace.lane_traces[lane_index]

    def step_forward(self) -> None:
        if self.result is None or not self.result.traces:
            return
        if self.current_step < len(self.result.traces) - 1:
            self.current_step += 1
        else:
            self.playing = False

    def step_back(self) -> None:
        if self.result is None or not self.result.traces:
            return
        self.current_step = max(0, self.current_step - 1)

    def cycle_lane(self, delta: int) -> None:
        trace = self.current_trace()
        if trace is None or not trace.lane_traces:
            return
        self.focus_lane = (self.focus_lane + delta) % len(trace.lane_traces)

    def handle_button(self, pos: tuple[int, int]) -> None:
        for name, button in self.buttons.items():
            if button.hit(pos):
                if name == "load":
                    chosen = _choose_file()
                    if chosen:
                        self.load_and_run(chosen)
                elif name == "run":
                    self.load_and_run(self.kernel_path)
                elif name == "play":
                    self.playing = not self.playing
                elif name == "step":
                    self.step_forward()
                elif name == "back":
                    self.step_back()
                elif name == "lane":
                    self.cycle_lane(1)

    def wrap_lines(self, text: str, width: int) -> list[str]:
        words = text.split()
        if not words:
            return [""]
        lines = [words[0]]
        for word in words[1:]:
            candidate = f"{lines[-1]} {word}"
            if self.small_font.size(candidate)[0] <= width:
                lines[-1] = candidate
            else:
                lines.append(word)
        return lines

    def draw_panel(self, rect: "pygame.Rect", title: str) -> None:
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=12)
        pygame.draw.rect(self.screen, (62, 73, 85), rect, width=1, border_radius=12)
        title_surface = self.body_font.render(title, True, TEXT)
        self.screen.blit(title_surface, (rect.x + 14, rect.y + 10))

    def draw_header(self) -> None:
        title = self.title_font.render("CUDA Architecture Visualizer", True, TEXT)
        self.screen.blit(title, (24, 68))
        status_color = WARNING if "failed" in self.message.lower() else MUTED
        status = self.small_font.render(self.message, True, status_color)
        self.screen.blit(status, (24, 104))
        for button in self.buttons.values():
            button.draw(self.screen, self.small_font)

        if self.result is not None:
            summary = (
                f"grid={self.result.config.grid_dim} block={self.result.config.block_dim} "
                f"warp={self.result.config.warp_size} "
                f"SMs={self.result.architecture.num_sms} "
                f"resident-CTAs/SM={self.result.architecture.max_blocks_per_sm}"
            )
            self.screen.blit(
                self.small_font.render(summary, True, ACCENT),
                (820, 32),
            )

    def draw_sm_overview(self, rect: "pygame.Rect", trace: WarpTrace | None) -> None:
        self.draw_panel(rect, "SM Overview")
        if trace is None:
            return
        sm_width = (rect.width - 32) // max(1, len(trace.sm_snapshots))
        for index, sm in enumerate(trace.sm_snapshots):
            x = rect.x + 14 + index * sm_width
            y = rect.y + 46
            box = pygame.Rect(x, y, sm_width - 10, rect.height - 60)
            pygame.draw.rect(self.screen, PANEL_ALT, box, border_radius=10)
            pygame.draw.rect(self.screen, (72, 85, 98), box, width=1, border_radius=10)
            header = f"SM {sm.sm_id}"
            if sm.sm_id == trace.sm_id:
                header += " issuing"
            self.screen.blit(self.small_font.render(header, True, TEXT), (x + 10, y + 10))
            self.screen.blit(
                self.small_font.render(f"{len(sm.resident_blocks)} resident CTAs", True, MUTED),
                (x + 10, y + 28),
            )
            block_y = y + 58
            for block in sm.resident_blocks[:6]:
                color = HIGHLIGHT if block.block_idx == sm.active_block else IDLE
                block_box = pygame.Rect(x + 10, block_y, box.width - 20, 44)
                pygame.draw.rect(self.screen, color, block_box, border_radius=8)
                pygame.draw.rect(self.screen, (50, 58, 68), block_box, width=1, border_radius=8)
                label = (
                    f"CTA {block.block_idx} | warp {block.next_warp}/{block.total_warps}"
                )
                self.screen.blit(self.small_font.render(label, True, BACKGROUND), (block_box.x + 8, block_box.y + 8))
                block_y += 52

    def draw_warp_lanes(self, rect: "pygame.Rect", trace: WarpTrace | None) -> None:
        self.draw_panel(rect, "Warp Lanes")
        if trace is None:
            return
        cols = min(8, max(1, len(trace.lane_traces)))
        tile_w = (rect.width - 28) // cols
        tile_h = 72
        for index, lane in enumerate(trace.lane_traces):
            row = index // cols
            col = index % cols
            x = rect.x + 14 + col * tile_w
            y = rect.y + 44 + row * (tile_h + 8)
            box = pygame.Rect(x, y, tile_w - 8, tile_h)
            color = HIGHLIGHT if index == self.focus_lane else ACTIVE
            pygame.draw.rect(self.screen, color, box, border_radius=8)
            pygame.draw.rect(self.screen, (44, 50, 58), box, width=1, border_radius=8)
            lines = [
                f"lane {lane.lane_id}",
                f"gidx {lane.global_linear_idx}",
                f"res {lane.result!r}",
            ]
            for line_index, line in enumerate(lines):
                self.screen.blit(
                    self.small_font.render(line, True, BACKGROUND),
                    (x + 8, y + 8 + line_index * 16),
                )

    def draw_scheduler(self, rect: "pygame.Rect", trace: WarpTrace | None) -> None:
        self.draw_panel(rect, "Scheduler + Stats")
        if trace is None or self.result is None:
            return
        lines = [
            f"kernel: {self.result.title}",
            f"step {trace.step} / {len(self.result.traces)}",
            f"cycle {trace.cycle}",
            f"issuing SM {trace.sm_id}",
            f"CTA {trace.block_idx}",
            f"warp {trace.warp_id}",
            f"active lanes {len(trace.active_lanes)}",
            f"global bytes {trace.stats['global_bytes_accessed']}",
            f"global transactions {trace.stats['global_transactions']}",
            f"shared accesses {trace.stats['shared_accesses']}",
            f"shared bank conflicts {trace.stats['shared_bank_conflicts']}",
            f"waves launched {self.result.stats.waves_launched}",
        ]
        y = rect.y + 46
        for line in lines:
            self.screen.blit(self.small_font.render(line, True, TEXT), (rect.x + 14, y))
            y += 18
        y += 6
        for line in self.wrap_lines(trace.scheduler_note, rect.width - 28):
            self.screen.blit(self.small_font.render(line, True, ACCENT), (rect.x + 14, y))
            y += 18

    def draw_memory_map(
        self,
        rect: "pygame.Rect",
        title: str,
        memory_words: dict[int, float],
        highlight_addrs: set[int],
    ) -> None:
        self.draw_panel(rect, title)
        if not memory_words:
            self.screen.blit(
                self.small_font.render("No memory activity yet.", True, MUTED),
                (rect.x + 14, rect.y + 46),
            )
            return
        items = list(memory_words.items())[:24]
        max_value = max(abs(value) for _, value in items) or 1.0
        for index, (word_addr, value) in enumerate(items):
            y = rect.y + 46 + index * 20
            addr = word_addr * 4
            color = HIGHLIGHT if addr in highlight_addrs else ACCENT
            width = int((abs(value) / max_value) * (rect.width - 170))
            bar_rect = pygame.Rect(rect.x + 90, y, rect.width - 150, 14)
            fill_rect = pygame.Rect(rect.x + 90, y, max(2, width), 14)
            pygame.draw.rect(self.screen, (40, 47, 56), bar_rect, border_radius=4)
            pygame.draw.rect(self.screen, color, fill_rect, border_radius=4)
            self.screen.blit(self.small_font.render(f"{addr:05d}", True, MUTED), (rect.x + 14, y - 1))
            self.screen.blit(self.small_font.render(f"{value:8.3f}", True, TEXT), (rect.x + rect.width - 60, y - 1))

    def draw_lane_details(self, rect: "pygame.Rect", lane: LaneTrace | None) -> None:
        self.draw_panel(rect, "Focused Lane")
        if lane is None:
            return
        lines = [
            f"lane {lane.lane_id}",
            f"thread {lane.thread_idx}",
            f"global idx {lane.global_linear_idx}",
            f"result {lane.result!r}",
        ]
        y = rect.y + 46
        for line in lines:
            self.screen.blit(self.small_font.render(line, True, TEXT), (rect.x + 14, y))
            y += 18
        y += 8
        self.screen.blit(self.small_font.render("registers", True, ACCENT), (rect.x + 14, y))
        y += 18
        for index, value in enumerate(lane.registers[:10]):
            self.screen.blit(
                self.small_font.render(f"r{index:02d} = {value:8.3f}", True, TEXT),
                (rect.x + 14, y),
            )
            y += 16
        y += 6
        self.screen.blit(self.small_font.render("memory ops", True, ACCENT), (rect.x + 14, y))
        y += 18
        for op in lane.memory_ops[:8]:
            line = f"{op.memory_space[:1].upper()} {op.op:<5} @{op.addr:05d} -> {op.value:.3f}"
            self.screen.blit(self.small_font.render(line, True, TEXT), (rect.x + 14, y))
            y += 16
        y += 6
        self.screen.blit(self.small_font.render("notes", True, ACCENT), (rect.x + 14, y))
        y += 18
        for note in lane.notes[:6]:
            for line in self.wrap_lines(note, rect.width - 28):
                self.screen.blit(self.small_font.render(line, True, MUTED), (rect.x + 14, y))
                y += 16

    def draw_footer(self) -> None:
        help_text = (
            "Keys: L load | R rerun | Space play/pause | N/Right next warp | "
            "B/Left back | ,/. lane | Up/Down speed"
        )
        self.screen.blit(
            self.small_font.render(help_text, True, MUTED),
            (24, WINDOW_HEIGHT - 28),
        )

    def render(self) -> None:
        self.screen.fill(BACKGROUND)
        self.draw_header()
        trace = self.current_trace()
        lane = self.current_lane(trace)
        highlight_addrs = set()
        if lane is not None:
            highlight_addrs = {op.addr for op in lane.memory_ops}

        self.draw_sm_overview(pygame.Rect(24, 140, 660, 300), trace)
        self.draw_warp_lanes(pygame.Rect(704, 140, 520, 300), trace)
        self.draw_scheduler(pygame.Rect(1244, 140, 232, 300), trace)
        self.draw_memory_map(
            pygame.Rect(24, 460, 480, 460),
            "Global Memory",
            {} if trace is None else trace.global_memory_words,
            highlight_addrs,
        )
        self.draw_memory_map(
            pygame.Rect(524, 460, 420, 460),
            "Shared Memory (Current CTA)",
            {} if trace is None else trace.shared_memory_words,
            highlight_addrs,
        )
        self.draw_lane_details(pygame.Rect(964, 460, 512, 460), lane)
        self.draw_footer()
        pygame.display.flip()

    def run(self) -> None:
        timer_accumulator = 0.0
        while True:
            dt = self.clock.tick(60) / 1000.0
            timer_accumulator += dt
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    self.handle_button(event.pos)
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_l:
                        chosen = _choose_file()
                        if chosen:
                            self.load_and_run(chosen)
                    elif event.key == pygame.K_r:
                        self.load_and_run(self.kernel_path)
                    elif event.key == pygame.K_SPACE:
                        self.playing = not self.playing
                    elif event.key in (pygame.K_n, pygame.K_RIGHT):
                        self.step_forward()
                    elif event.key in (pygame.K_b, pygame.K_LEFT):
                        self.step_back()
                    elif event.key == pygame.K_PERIOD:
                        self.cycle_lane(1)
                    elif event.key == pygame.K_COMMA:
                        self.cycle_lane(-1)
                    elif event.key == pygame.K_UP:
                        self.playback_fps = min(30, self.playback_fps + 1)
                    elif event.key == pygame.K_DOWN:
                        self.playback_fps = max(1, self.playback_fps - 1)

            if self.playing and self.result is not None and self.result.traces:
                if timer_accumulator >= 1.0 / self.playback_fps:
                    timer_accumulator = 0.0
                    self.step_forward()
            self.render()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="CUDA architecture visualizer")
    parser.add_argument("kernel_file", nargs="?", help="Path to a Python kernel file")
    args = parser.parse_args(argv)

    try:
        app = SimulatorGUI(args.kernel_file)
        app.run()
    except Exception as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
