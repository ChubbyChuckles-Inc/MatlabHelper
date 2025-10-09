"""Service responsible for injecting MATLAB code into the editor window."""

from __future__ import annotations

import contextlib
import time
from typing import Optional

try:
    import win32clipboard
    import win32con
except ImportError:  # pragma: no cover - exercised through dependency injection in tests
    win32clipboard = None  # type: ignore[assignment]
    win32con = None  # type: ignore[assignment]

import keyboard

from src.config.constants import INJECTION_PRE_FOCUS_DELAY_SECONDS
from src.models.matlab_window import MatlabWindowInfo
from src.services.window_service import MatlabWindowDetectionError, MatlabWindowService


class MatlabInjectionError(RuntimeError):
    """Raised when content injection into MATLAB fails."""


class MatlabInjector:
    """Inject MATLAB script content by simulating keyboard input and clipboard pastes."""

    def __init__(
        self,
        window_service: MatlabWindowService,
        *,
        keyboard_module=keyboard,
        clipboard_module=win32clipboard,
        con_module=win32con,
        sleep_fn=time.sleep,
    ) -> None:
        self._window_service = window_service
        self._keyboard = keyboard_module
        self._clipboard = clipboard_module
        self._con = con_module
        self._sleep = sleep_fn

    def inject(self, window: MatlabWindowInfo, content: str) -> None:
        """Focus *window* and stream *content* into it."""

        if not content:
            raise MatlabInjectionError("MATLAB content is empty; nothing to inject.")
        try:
            self._window_service.focus_window(window)
        except MatlabWindowDetectionError as exc:  # pragma: no cover - thin wrapper
            raise MatlabInjectionError(str(exc)) from exc

        self._sleep(INJECTION_PRE_FOCUS_DELAY_SECONDS)
        self._write_via_clipboard(content)

    # Internal helpers -------------------------------------------------

    def _write_via_clipboard(self, content: str) -> None:
        if self._clipboard is None or self._con is None:
            raise MatlabInjectionError(
                "pywin32 clipboard bindings are required for MATLAB content injection."
            )

        previous_text: Optional[str] = None
        try:
            self._clipboard.OpenClipboard()
            with contextlib.suppress(Exception):
                previous_text = str(self._clipboard.GetClipboardData(self._con.CF_UNICODETEXT))
            self._clipboard.EmptyClipboard()
            self._clipboard.SetClipboardText(content, self._con.CF_UNICODETEXT)
        except Exception as exc:  # pragma: no cover - pywin32 specific failure
            raise MatlabInjectionError("Failed to copy content to clipboard.") from exc
        finally:
            with contextlib.suppress(Exception):
                self._clipboard.CloseClipboard()

        # Send paste sequence
        self._keyboard.send("ctrl+a")
        self._sleep(0.05)
        self._keyboard.send("delete")
        self._sleep(0.05)
        self._keyboard.send("ctrl+v")
        self._sleep(0.05)

        if previous_text:
            self._restore_clipboard(previous_text)

    def _restore_clipboard(self, text: str) -> None:
        if self._clipboard is None or self._con is None:
            return
        try:
            self._clipboard.OpenClipboard()
            self._clipboard.EmptyClipboard()
            self._clipboard.SetClipboardText(text, self._con.CF_UNICODETEXT)
        except Exception:
            pass
        finally:
            with contextlib.suppress(Exception):
                self._clipboard.CloseClipboard()
