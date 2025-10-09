"""Services that interact with Windows APIs to locate MATLAB editor windows."""

from __future__ import annotations

import sys
from typing import Callable, Optional

try:
    import win32con
    import win32gui
except ImportError:  # pragma: no cover - exercised through dependency injection in tests
    win32con = None  # type: ignore[assignment]
    win32gui = None  # type: ignore[assignment]

from src.config.constants import MATLAB_WINDOW_KEYWORDS
from src.models.matlab_window import MatlabWindowInfo


class MatlabWindowDetectionError(RuntimeError):
    """Raised when the MATLAB editor window cannot be located."""


class MatlabWindowService:
    """Locate and focus MATLAB editor windows on Windows platforms."""

    def __init__(
        self,
        *,
        gui_module: Optional[object] = None,
        con_module: Optional[object] = None,
        platform_checker: Callable[[], str] = (lambda: sys.platform),
    ) -> None:
        self._gui = gui_module if gui_module is not None else win32gui
        self._con = con_module if con_module is not None else win32con
        self._platform_checker = platform_checker

    # We avoid property for test clarity

    def is_supported(self) -> bool:
        """Return ``True`` if running on Windows with win32 bindings available."""

        return (
            self._platform_checker() == "win32" and self._gui is not None and self._con is not None
        )

    def get_active_matlab_window(self) -> Optional[MatlabWindowInfo]:
        """Return the currently focused MATLAB editor window, if available."""

        if not self.is_supported():
            raise MatlabWindowDetectionError(
                "MATLAB window detection is only supported on Windows."
            )

        assert self._gui is not None  # for mypy/static type checkers
        hwnd = self._gui.GetForegroundWindow()
        if hwnd == 0:
            return None

        title = str(self._gui.GetWindowText(hwnd))
        if not any(keyword.lower() in title.lower() for keyword in MATLAB_WINDOW_KEYWORDS):
            return None
        return MatlabWindowInfo(handle=hwnd, title=title.strip())

    def focus_window(self, window: MatlabWindowInfo) -> None:
        """Bring *window* to the foreground before injection."""

        if not self.is_supported():
            raise MatlabWindowDetectionError("Window focus is only supported on Windows.")

        assert self._gui is not None and self._con is not None
        if not window.is_valid():
            raise MatlabWindowDetectionError("Cannot focus an invalid MATLAB window.")
        try:
            self._gui.ShowWindow(window.handle, self._con.SW_RESTORE)
            self._gui.SetForegroundWindow(window.handle)
        except Exception as exc:  # pragma: no cover - rare win32 specific errors
            raise MatlabWindowDetectionError("Failed to focus MATLAB editor window.") from exc
