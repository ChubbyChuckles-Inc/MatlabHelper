"""Services that interact with Windows APIs to locate MATLAB editor windows."""

from __future__ import annotations

import sys
from typing import Callable, List, Optional, Tuple

try:
    import win32con
    import win32gui
except ImportError:  # pragma: no cover - exercised through dependency injection in tests
    win32con = None  # type: ignore[assignment]
    win32gui = None  # type: ignore[assignment]


def _load_ctypes_win32() -> Tuple[Optional[object], Optional[object]]:
    """Return ctypes-based replacements for win32gui/win32con when unavailable."""

    try:
        import ctypes
        from ctypes import wintypes
    except Exception:  # pragma: no cover - missing ctypes on exotic interpreters
        return None, None

    try:
        user32 = ctypes.windll.user32
    except AttributeError:  # pragma: no cover - non-Windows platforms
        return None, None

    class _CTypesWin32Gui:
        def __init__(self) -> None:
            self._user32 = user32
            self._enum_proc = None

        def GetForegroundWindow(self) -> int:
            return int(self._user32.GetForegroundWindow())

        def GetWindowText(self, hwnd: int) -> str:
            length = int(self._user32.GetWindowTextLengthW(hwnd))
            buffer = ctypes.create_unicode_buffer(length + 2)
            self._user32.GetWindowTextW(hwnd, buffer, len(buffer))
            return buffer.value

        def EnumWindows(self, callback, param) -> None:
            cmp_func = ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND, wintypes.LPARAM)

            def _proc(hwnd, lparam) -> bool:  # pragma: no cover - ctypes glue
                return bool(callback(int(hwnd), int(lparam)))

            self._enum_proc = cmp_func(_proc)
            self._user32.EnumWindows(self._enum_proc, param)

        def ShowWindow(self, hwnd: int, flag: int) -> None:
            self._user32.ShowWindow(hwnd, flag)

        def SetForegroundWindow(self, hwnd: int) -> None:
            self._user32.SetForegroundWindow(hwnd)

    class _CTypesWin32Con:
        SW_RESTORE = 9

    return _CTypesWin32Gui(), _CTypesWin32Con()


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
        default_gui = win32gui
        default_con = win32con
        if default_gui is None or default_con is None:
            fallback_gui, fallback_con = _load_ctypes_win32()
            if default_gui is None and fallback_gui is not None:
                default_gui = fallback_gui
            if default_con is None and fallback_con is not None:
                default_con = fallback_con

        self._gui = gui_module if gui_module is not None else default_gui
        self._con = con_module if con_module is not None else default_con
        self._platform_checker = platform_checker

    # We avoid property for test clarity

    def is_supported(self) -> bool:
        """Return ``True`` if running on Windows with win32 bindings available."""

        return (
            self._platform_checker() == "win32" and self._gui is not None and self._con is not None
        )

    def get_active_matlab_window(self) -> Optional[MatlabWindowInfo]:
        """Return the currently focused top-level window, if available."""

        if not self.is_supported():
            raise MatlabWindowDetectionError(
                "MATLAB window detection is only supported on Windows."
            )

        assert self._gui is not None  # for mypy/static type checkers
        hwnd = self._gui.GetForegroundWindow()
        if hwnd == 0:
            return None

        title = str(self._gui.GetWindowText(hwnd)).strip()
        if not title:
            return None
        return MatlabWindowInfo(handle=hwnd, title=title)

    def list_matlab_windows(self) -> List[MatlabWindowInfo]:
        """Enumerate all open top-level windows with a title."""

        if not self.is_supported():
            raise MatlabWindowDetectionError(
                "MATLAB window detection is only supported on Windows."
            )

        assert self._gui is not None

        windows: List[MatlabWindowInfo] = []
        seen: set[int] = set()

        def _callback(hwnd: int, _param: int) -> bool:
            if hwnd in seen:
                return True
            title = str(self._gui.GetWindowText(hwnd)).strip()
            if not title:
                return True
            windows.append(MatlabWindowInfo(handle=hwnd, title=title))
            seen.add(hwnd)
            return True

        try:
            self._gui.EnumWindows(_callback, 0)
        except Exception as exc:  # pragma: no cover - rare win32 enumeration failure
            raise MatlabWindowDetectionError("Failed to enumerate windows.") from exc

        windows.sort(key=lambda info: info.title.lower())
        return windows

    def focus_window(self, window: MatlabWindowInfo) -> None:
        """Bring *window* to the foreground before injection."""

        if not self.is_supported():
            raise MatlabWindowDetectionError("Window focus is only supported on Windows.")

        assert self._gui is not None and self._con is not None
        if not window.is_valid():
            raise MatlabWindowDetectionError("Cannot focus an invalid window.")
        try:
            self._gui.ShowWindow(window.handle, self._con.SW_RESTORE)
            self._gui.SetForegroundWindow(window.handle)
        except Exception as exc:  # pragma: no cover - rare win32 specific errors
            raise MatlabWindowDetectionError("Failed to focus target window.") from exc
