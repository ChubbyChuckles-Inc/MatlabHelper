"""Service responsible for injecting MATLAB code into the editor window."""

from __future__ import annotations

import random
import string
import time
from typing import Optional, Tuple

import keyboard

from src.config.constants import (
    INJECTION_PRE_FOCUS_DELAY_SECONDS,
    MAX_CHAR_DELAY_SECONDS,
    MIN_CHAR_DELAY_SECONDS,
    TEMPO_MAX_FACTOR,
    TEMPO_MIN_FACTOR,
    TEMPO_REFERENCE_SECONDS,
    TYPING_ERROR_MAX_BURST,
    TYPING_ERROR_MIN_BURST,
    TYPING_ERROR_PROBABILITY,
)
from src.models.matlab_window import MatlabWindowInfo
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
    ) -> None:
        self._window_service = window_service
        self._keyboard = keyboard_module
        self._sleep = sleep_fn
        self._rng = rng or random.Random()

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

        self._focus_window(window)
        self._sleep(INJECTION_PRE_FOCUS_DELAY_SECONDS)

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
        base_min, base_max = MIN_CHAR_DELAY_SECONDS, MAX_CHAR_DELAY_SECONDS
        if tempo_hint is None:
            return base_min, base_max

        tempo = max(tempo_hint, 1e-3)
        ratio = tempo / TEMPO_REFERENCE_SECONDS
        factor = min(max(ratio, TEMPO_MIN_FACTOR), TEMPO_MAX_FACTOR)
        return base_min * factor, base_max * factor

    def _type_character_with_variation(self, char: str, delay_range: Tuple[float, float]) -> None:
        if char not in {"\n", "\r", "\t"} and self._should_simulate_error():
            mistakes = self._rng.randint(TYPING_ERROR_MIN_BURST, TYPING_ERROR_MAX_BURST)
            for _ in range(mistakes):
                typo = self._random_typo_character()
                self._keyboard.write(typo, delay=0)
                self._sleep(self._rng.uniform(*delay_range))
            for _ in range(mistakes):
                self._keyboard.send("backspace")
                self._sleep(self._rng.uniform(*delay_range))

        self._type_character(char)
        if char != "\r":
            self._sleep(self._rng.uniform(*delay_range))

    def _should_simulate_error(self) -> bool:
        return self._rng.random() < TYPING_ERROR_PROBABILITY

    def _random_typo_character(self) -> str:
        population = string.ascii_letters + string.digits + string.punctuation
        return self._rng.choice(population)

    def _type_character(self, char: str) -> None:
        if char == "\r":
            return
        if char == "\n":
            self._keyboard.send("enter")
        elif char == "\t":
            self._keyboard.send("tab")
        else:
            self._keyboard.write(char, delay=0)
