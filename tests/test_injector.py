"""Tests for the MatlabInjector typo correction flow."""

from __future__ import annotations

from dataclasses import replace

from src.models.matlab_window import MatlabWindowInfo
from src.models.typing_settings import TypingSettings
from src.services.injector_service import MatlabInjector


class _StubWindowService:
    def __init__(self) -> None:
        self.focus_calls: list[MatlabWindowInfo] = []

    def focus_window(self, window: MatlabWindowInfo) -> None:
        self.focus_calls.append(window)


class _KeyboardStub:
    def __init__(self) -> None:
        self.events: list[tuple[str, str]] = []

    def write(
        self, text: str, delay: float = 0
    ) -> None:  # pragma: no cover - behaviour checked via events
        self.events.append(("write", text))

    def send(self, key: str) -> None:  # pragma: no cover - behaviour checked via events
        self.events.append(("send", key))


class _DeterministicRandom:
    def __init__(self) -> None:
        self._random_values = [0.0] + [0.9] * 10
        self._randint_values = [2]
        self._choice_values = ["s"]

    def random(self) -> float:
        return self._random_values.pop(0)

    def randint(self, a: int, b: int) -> int:
        return self._randint_values.pop(0)

    def choice(self, population):
        if self._choice_values:
            return self._choice_values.pop(0)
        return population[0]

    def uniform(self, a: float, b: float) -> float:
        return 0.0


def test_typo_sequence_requires_user_keypresses() -> None:
    service = _StubWindowService()
    keyboard = _KeyboardStub()
    rng = _DeterministicRandom()
    base_settings = TypingSettings.from_defaults()
    settings = replace(
        base_settings,
        min_char_delay=0.0,
        max_char_delay=0.0,
        error_probability=0.5,
        error_min_burst=2,
        error_max_burst=2,
    ).clamped()

    injector = MatlabInjector(
        service,
        keyboard_module=keyboard,
        sleep_fn=lambda _delay: None,
        rng=rng,
        settings=settings,
    )

    window = MatlabWindowInfo(handle=1, title="MATLAB Editor")

    keystrokes = [
        ("a", "x"),
        ("b", "y"),
        ("c", "z"),
        ("d", "p"),
        ("e", "q"),
        ("f", "r"),
        ("g", "s"),
    ]

    for index, (char, source) in enumerate(keystrokes, start=1):
        injector.type_text(window, char, source_inputs=(source,))
        assert len(keyboard.events) == index

    assert keyboard.events == [
        ("write", "x"),
        ("write", "y"),
        ("send", "backspace"),
        ("send", "backspace"),
        ("write", "a"),
        ("write", "b"),
        ("write", "c"),
    ]
