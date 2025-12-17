"""Tests for the Qt-aware keyboard monitor."""

from __future__ import annotations

from types import SimpleNamespace

from src.services.keyboard_monitor import KeyboardMonitor


class FakeKeyboard:
    def __init__(self) -> None:
        self.callback = None
        self.unhook_calls = 0
        self.suppress_values: list[bool] = []

    def hook(self, callback, *, suppress: bool = False):  # type: ignore[override]
        if self.callback is not None:
            raise AssertionError("hook called twice without unhook")
        self.callback = callback
        self.suppress_values.append(suppress)
        return callback

    def unhook(self, handle) -> None:  # type: ignore[override]
        if handle != self.callback:
            raise AssertionError("unexpected handle passed to unhook")
        self.callback = None
        self.unhook_calls += 1


def test_keyboard_monitor_emits_signals(qtbot) -> None:
    fake_keyboard = FakeKeyboard()
    monitor = KeyboardMonitor(keyboard_module=fake_keyboard)

    with qtbot.waitSignal(monitor.listening_changed, timeout=500) as blocker:
        monitor.start()
    assert blocker.args == [True]
    assert fake_keyboard.suppress_values == [True]

    assert fake_keyboard.callback is not None

    with qtbot.waitSignal(monitor.key_pressed, timeout=500) as key_signal:
        fake_keyboard.callback(SimpleNamespace(name="space"))
    assert key_signal.args == ["space"]

    assert monitor.is_listening()

    with qtbot.waitSignal(monitor.listening_changed, timeout=500) as blocker:
        monitor.stop()
    assert blocker.args == [False]
    assert not monitor.is_listening()
    assert fake_keyboard.unhook_calls == 1


def test_keyboard_monitor_passthrough_toggle(qtbot) -> None:
    fake_keyboard = FakeKeyboard()
    monitor = KeyboardMonitor(keyboard_module=fake_keyboard)

    monitor.start()
    assert fake_keyboard.suppress_values == [True]

    monitor.set_passthrough_enabled(True)
    assert fake_keyboard.unhook_calls == 1
    assert fake_keyboard.suppress_values[-1] is False

    monitor.set_passthrough_enabled(True)
    assert fake_keyboard.unhook_calls == 1  # no change when already enabled

    monitor.set_passthrough_enabled(False)
    assert fake_keyboard.unhook_calls == 2
    assert fake_keyboard.suppress_values[-1] is True

    monitor.stop()
    assert fake_keyboard.unhook_calls == 3
