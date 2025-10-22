"""Tests for MATLAB window detection service."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Optional

import pytest

from src.models.matlab_window import MatlabWindowInfo
from src.services.window_service import (
    MatlabWindowDetectionError,
    MatlabWindowService,
)


class FakeWin32Gui:
    def __init__(self, hwnd: int, title: str, windows: Optional[dict[int, str]] = None) -> None:
        self._hwnd = hwnd
        self._title = title
        self._windows: dict[int, str] = dict(windows or {})
        self._windows[hwnd] = title
        self.focused: list[int] = []

    def GetForegroundWindow(self) -> int:
        return self._hwnd

    def GetWindowText(self, hwnd: int) -> str:
        return self._windows.get(hwnd, "")

    def ShowWindow(self, hwnd: int, flag: int) -> None:
        self.focused.append(hwnd)

    def SetForegroundWindow(self, hwnd: int) -> None:
        self.focused.append(hwnd)

    def EnumWindows(self, callback, param) -> None:
        for handle in list(self._windows):
            if not callback(handle, param):
                break


class FakeWin32Con(SimpleNamespace):
    pass


def test_get_active_matlab_window_returns_info() -> None:
    gui = FakeWin32Gui(42, "MATLAB Editor - demo.m")
    con = FakeWin32Con(SW_RESTORE=9)
    service = MatlabWindowService(gui_module=gui, con_module=con, platform_checker=lambda: "win32")

    window = service.get_active_matlab_window()

    assert isinstance(window, MatlabWindowInfo)
    assert window and window.handle == 42
    assert gui.focused == []


def test_get_active_matlab_window_returns_none_when_title_missing() -> None:
    gui = FakeWin32Gui(42, "")
    con = FakeWin32Con(SW_RESTORE=9)
    service = MatlabWindowService(gui_module=gui, con_module=con, platform_checker=lambda: "win32")

    assert service.get_active_matlab_window() is None


def test_focus_window_invokes_win32_calls() -> None:
    gui = FakeWin32Gui(42, "MATLAB Editor")
    con = FakeWin32Con(SW_RESTORE=9)
    service = MatlabWindowService(gui_module=gui, con_module=con, platform_checker=lambda: "win32")
    window = MatlabWindowInfo(handle=42, title="MATLAB Editor")

    service.focus_window(window)

    assert gui.focused == [42, 42]


def test_list_matlab_windows_returns_all_titled_windows() -> None:
    windows = {
        10: "MATLAB Editor - demo.m",
        11: "Notepad",
        12: "",
        13: "PowerShell",
    }
    gui = FakeWin32Gui(10, "MATLAB Editor - demo.m", windows=windows)
    con = FakeWin32Con(SW_RESTORE=9)
    service = MatlabWindowService(gui_module=gui, con_module=con, platform_checker=lambda: "win32")

    enumerated = service.list_matlab_windows()

    titles = [info.title for info in enumerated]
    assert "MATLAB Editor - demo.m" in titles
    assert "Notepad" in titles
    assert "PowerShell" in titles
    assert all(info.handle in {10, 11, 13} for info in enumerated)


def test_detection_raises_on_unsupported_platform() -> None:
    service = MatlabWindowService(platform_checker=lambda: "linux")

    with pytest.raises(MatlabWindowDetectionError):
        service.get_active_matlab_window()
