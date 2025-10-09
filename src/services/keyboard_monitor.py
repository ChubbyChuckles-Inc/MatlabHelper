"""Global keyboard monitor that emits signals when random keys are pressed."""

from __future__ import annotations

from typing import Optional

from PyQt6 import QtCore

import keyboard


class KeyboardMonitorError(RuntimeError):
    """Raised when the global keyboard monitor cannot start or stop."""


class KeyboardMonitor(QtCore.QObject):
    """Qt-aware wrapper around the :mod:`keyboard` global hook."""

    key_pressed = QtCore.pyqtSignal(str)
    listening_changed = QtCore.pyqtSignal(bool)

    def __init__(self, *, keyboard_module=keyboard) -> None:
        super().__init__()
        self._keyboard = keyboard_module
        self._hook: Optional[object] = None

    def is_listening(self) -> bool:
        return self._hook is not None

    @QtCore.pyqtSlot()
    def start(self) -> None:
        if self.is_listening():
            return
        try:
            self._hook = self._keyboard.hook(self._handle_event)
        except Exception as exc:  # pragma: no cover - library level failure
            raise KeyboardMonitorError("Failed to start keyboard hook.") from exc
        self.listening_changed.emit(True)

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        if not self.is_listening():
            return
        try:
            self._keyboard.unhook(self._hook)
        except Exception as exc:  # pragma: no cover - library level failure
            raise KeyboardMonitorError("Failed to stop keyboard hook.") from exc
        finally:
            self._hook = None
        self.listening_changed.emit(False)

    def _handle_event(self, event) -> None:  # type: ignore[override]
        # The keyboard library sends a KeyboardEvent. We only need the name attribute.
        key_name = getattr(event, "name", "") or "unknown"
        self.key_pressed.emit(str(key_name))
        self.stop()
