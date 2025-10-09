"""Tests for MATLAB window detection service."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from src.models.matlab_window import MatlabWindowInfo
from src.services.window_service import (
    MatlabWindowDetectionError,
    MatlabWindowService,
)


class FakeWin32Gui:
    def __init__(self, hwnd: int, title: str) -> None:
        self._hwnd = hwnd
        self._title = title
        self.focused: list[int] = []

    def GetForegroundWindow(self) -> int:
        return self._hwnd

    def GetWindowText(self, hwnd: int) -> str:
        if hwnd != self._hwnd:
            raise AssertionError("Unexpected handle")
        return self._title

    def ShowWindow(self, hwnd: int, flag: int) -> None:
        self.focused.append(hwnd)

    def SetForegroundWindow(self, hwnd: int) -> None:
        self.focused.append(hwnd)


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


def test_get_active_matlab_window_returns_none_when_not_matlab() -> None:
    gui = FakeWin32Gui(42, "Notepad")
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


def test_detection_raises_on_unsupported_platform() -> None:
    service = MatlabWindowService(platform_checker=lambda: "linux")

    with pytest.raises(MatlabWindowDetectionError):
        service.get_active_matlab_window()
