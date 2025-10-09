"""Service responsible for injecting MATLAB code into the editor window."""

from __future__ import annotations

import random
import string
import time
from collections import deque
from typing import Deque, Dict, Iterable, Optional, Sequence, Tuple

import keyboard

from src.models.matlab_window import MatlabWindowInfo
from src.models.typing_settings import TypingSettings
from src.services.window_service import MatlabWindowDetectionError, MatlabWindowService


class MatlabInjectionError(RuntimeError):
    """Raised when content injection into MATLAB fails."""


class MatlabInjector:
    """Inject MATLAB script content by simulating keystrokes."""

    def __init__(
        self,
        window_service: MatlabWindowService,
        *,
        keyboard_module=keyboard,
        sleep_fn=time.sleep,
        rng: Optional[random.Random] = None,
        settings: Optional[TypingSettings] = None,
    ) -> None:
        self._window_service = window_service
        self._keyboard = keyboard_module
        self._sleep = sleep_fn
        self._rng = rng or random.Random()
        self._settings = (settings or TypingSettings.from_defaults()).clamped()
        self._last_window_handle: Optional[int] = None
        self._pending_backspaces = 0
        self._pending_chars = deque()  # type: Deque[str]

    def type_text(
        self,
        window: MatlabWindowInfo,
        text: str,
        *,
        tempo_hint: Optional[float] = None,
    ) -> None:
        """Focus *window* and type *text* character by character.

        Parameters
        ----------
        window:
            Target MATLAB editor window.
        text:
            Characters to type in order.
        tempo_hint:
            Optional duration (in seconds) since the previous triggering key event. Used to
            bias the typing cadence so that quick keyboard presses yield faster typing and
            slower presses feel more deliberate.
        """

        if not text:
            return

        if window.handle != self._last_window_handle:
            self._focus_window(window)
            if self._settings.focus_delay > 0:
                self._sleep(self._settings.focus_delay)
            self._last_window_handle = window.handle

        try:
            delay_range = self._compute_delay_range(tempo_hint)
            for char in text:
                self._type_character_with_variation(char, delay_range)
        except Exception as exc:  # pragma: no cover - keyboard library failure
            raise MatlabInjectionError("Failed to type characters into MATLAB editor.") from exc

    def inject(self, window: MatlabWindowInfo, content: str) -> None:
        """Compatibility wrapper around :meth:`type_text`."""

        self.type_text(window, content)

    # Internal helpers -------------------------------------------------

    def _focus_window(self, window: MatlabWindowInfo) -> None:
        try:
            self._window_service.focus_window(window)
        except MatlabWindowDetectionError as exc:  # pragma: no cover - thin wrapper
            raise MatlabInjectionError(str(exc)) from exc

    def _compute_delay_range(self, tempo_hint: Optional[float]) -> Tuple[float, float]:
        settings = self._settings
        base_min, base_max = settings.min_char_delay, settings.max_char_delay
        if tempo_hint is None:
            return base_min, base_max

        tempo = max(tempo_hint, 1e-6)
        ratio = tempo / settings.tempo_reference_seconds
        factor = min(max(ratio, settings.tempo_min_factor), settings.tempo_max_factor)
        scaled_min = base_min * factor
        scaled_max = max(scaled_min, base_max * factor)
        return scaled_min, scaled_max

    def _type_character_with_variation(self, char: str, delay_range: Tuple[float, float]) -> None:
        self._flush_pending_corrections(delay_range)

        if char not in {"\n", "\r", "\t"} and self._should_simulate_error():
            mistakes = self._rng.randint(
                self._settings.error_min_burst, self._settings.error_max_burst
            )
            for _ in range(mistakes):
                typo = self._random_typo_character(char)
                self._keyboard.write(typo, delay=0)
                self._sleep_random(delay_range)
            self._pending_backspaces += mistakes
            self._pending_chars.append(char)
            return

        self._type_character(char)
        if char != "\r":
            self._sleep_random(delay_range)

    def _flush_pending_corrections(self, delay_range: Tuple[float, float]) -> None:
        while self._pending_backspaces > 0:
            self._keyboard.send("backspace")
            self._sleep_random(delay_range)
            self._pending_backspaces -= 1
        while self._pending_chars:
            correction = self._pending_chars.popleft()
            self._type_character(correction)
            if correction != "\r":
                self._sleep_random(delay_range)

    def _sleep_random(self, delay_range: Tuple[float, float]) -> None:
        start, end = delay_range
        if end <= 0:
            return
        delay = self._rng.uniform(max(0.0, start), max(start, end))
        if delay > 0:
            self._sleep(delay)

    def _should_simulate_error(self) -> bool:
        return self._rng.random() < self._settings.error_probability

    def _random_typo_character(self, target: str) -> str:
        neighbors = _NEAR_KEY_LOOKUP.get(target.lower())
        if not neighbors:
            population = string.ascii_lowercase + string.digits
            return self._rng.choice(population)
        choice = self._rng.choice(neighbors)
        if target.isupper():
            return choice.upper()
        if target in _SHIFTED_TO_BASE:
            shifted = _BASE_TO_SHIFTED.get(choice)
            if shifted:
                return shifted
        return choice

    def _type_character(self, char: str) -> None:
        if char == "\r":
            return
        if char == "\n":
            self._keyboard.send("enter")
        elif char == "\t":
            self._keyboard.send("tab")
        else:
            self._keyboard.write(char, delay=0)

    def update_settings(self, settings: TypingSettings) -> None:
        self._settings = settings.clamped()

    def reset_focus_cache(self) -> None:
        self._last_window_handle = None


def _build_neighbor_lookup(rows: Sequence[str]) -> Dict[str, Tuple[str, ...]]:
    grid = [(idx, list(row)) for idx, row in enumerate(rows)]
    positions: Dict[str, Tuple[int, int]] = {}
    for row_idx, row_chars in grid:
        for col_idx, char in enumerate(row_chars):
            positions[char] = (row_idx, col_idx)

    def neighbors(char: str) -> Tuple[str, ...]:
        if char not in positions:
            return tuple()
        row_idx, col_idx = positions[char]
        results: set[str] = set()
        for r in range(row_idx - 1, row_idx + 2):
            if r < 0 or r >= len(grid):
                continue
            row_chars = grid[r][1]
            for c in range(col_idx - 1, col_idx + 2):
                if c < 0 or c >= len(row_chars):
                    continue
                neighbor_char = row_chars[c]
                if neighbor_char != char:
                    results.add(neighbor_char)
        return tuple(sorted(results))

    lookup: Dict[str, Tuple[str, ...]] = {char: neighbors(char) for char in positions}
    return lookup


_QWERTY_ROWS: Tuple[str, ...] = (
    "`1234567890-=",
    "qwertyuiop[]\\",
    "asdfghjkl;'",
    "zxcvbnm,./",
    " ",
)

_SHIFTED_TO_BASE: Dict[str, str] = {
    "~": "`",
    "!": "1",
    "@": "2",
    "#": "3",
    "$": "4",
    "%": "5",
    "^": "6",
    "&": "7",
    "*": "8",
    "(": "9",
    ")": "0",
    "_": "-",
    "+": "=",
    "{": "[",
    "}": "]",
    "|": "\\",
    ":": ";",
    '"': "'",
    "<": ",",
    ">": ".",
    "?": "/",
}

_BASE_TO_SHIFTED: Dict[str, str] = {base: shifted for shifted, base in _SHIFTED_TO_BASE.items()}

_NEAR_KEY_LOOKUP: Dict[str, Tuple[str, ...]] = _build_neighbor_lookup(_QWERTY_ROWS)
