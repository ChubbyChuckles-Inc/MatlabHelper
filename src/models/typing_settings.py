"""Runtime-configurable typing settings for the injector."""

from __future__ import annotations

from dataclasses import dataclass, replace

from src.config import constants


@dataclass(frozen=True)
class TypingSettings:
    focus_delay: float
    min_char_delay: float
    max_char_delay: float
    tempo_reference_seconds: float
    tempo_min_factor: float
    tempo_max_factor: float
    error_probability: float
    error_min_burst: int
    error_max_burst: int

    @classmethod
    def from_defaults(cls) -> "TypingSettings":
        return cls(
            focus_delay=constants.INJECTION_PRE_FOCUS_DELAY_SECONDS,
            min_char_delay=constants.MIN_CHAR_DELAY_SECONDS,
            max_char_delay=constants.MAX_CHAR_DELAY_SECONDS,
            tempo_reference_seconds=constants.TEMPO_REFERENCE_SECONDS,
            tempo_min_factor=constants.TEMPO_MIN_FACTOR,
            tempo_max_factor=constants.TEMPO_MAX_FACTOR,
            error_probability=constants.TYPING_ERROR_PROBABILITY,
            error_min_burst=constants.TYPING_ERROR_MIN_BURST,
            error_max_burst=constants.TYPING_ERROR_MAX_BURST,
        )

    def clamped(self) -> "TypingSettings":
        min_delay = max(0.0, min(self.min_char_delay, self.max_char_delay))
        max_delay = max(min_delay, self.max_char_delay)
        min_factor = max(0.0, self.tempo_min_factor)
        max_factor = max(min_factor if min_factor > 0 else 0.01, self.tempo_max_factor)
        probability = min(max(self.error_probability, 0.0), 1.0)
        min_burst = max(0, self.error_min_burst)
        max_burst = max(min_burst, self.error_max_burst)
        focus_delay = max(0.0, self.focus_delay)
        tempo_reference = max(0.001, self.tempo_reference_seconds)
        return replace(
            self,
            min_char_delay=min_delay,
            max_char_delay=max_delay,
            tempo_min_factor=min_factor,
            tempo_max_factor=max_factor,
            error_probability=probability,
            error_min_burst=min_burst,
            error_max_burst=max_burst,
            focus_delay=focus_delay,
            tempo_reference_seconds=tempo_reference,
        )
