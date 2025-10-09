"""Integration tests for the Qt controller using stubbed services."""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from PyQt6 import QtCore, QtWidgets

from src.app.controller import MainController
from src.models.matlab_window import MatlabWindowInfo
from src.ui.main_window import MatlabHelperMainWindow


class StubWindowService:
    def __init__(self) -> None:
        self.window = MatlabWindowInfo(handle=99, title="MATLAB Editor - demo")
        self.focus_calls = 0

    def get_active_matlab_window(self) -> MatlabWindowInfo:
        return self.window

    def list_matlab_windows(self) -> list[MatlabWindowInfo]:
        return [self.window]

    def focus_window(self, window: MatlabWindowInfo) -> None:
        assert window == self.window
        self.focus_calls += 1


class StubInjector:
    def __init__(self, focus_hook: Callable[[MatlabWindowInfo], None]) -> None:
        self.calls: list[tuple[MatlabWindowInfo, str, float | None, tuple[str, ...] | None]] = []
        self._focus_hook = focus_hook

    def type_text(
        self,
        window: MatlabWindowInfo,
        content: str,
        *,
        tempo_hint: float | None = None,
        source_inputs: tuple[str, ...] | None = None,
    ) -> None:
        self._focus_hook(window)
        self.calls.append((window, content, tempo_hint, source_inputs))

    def inject(self, window: MatlabWindowInfo, content: str) -> None:
        self.type_text(window, content)


class StubKeyboardMonitor(QtCore.QObject):
    key_pressed = QtCore.pyqtSignal(str)
    listening_changed = QtCore.pyqtSignal(bool)

    def __init__(self) -> None:
        super().__init__()
        self.started = False

    def start(self) -> None:
        self.started = True
        self.listening_changed.emit(True)

    def stop(self) -> None:
        if self.started:
            self.started = False
            self.listening_changed.emit(False)

    def fire(self, key_name: str) -> None:
        self.key_pressed.emit(key_name)


def test_controller_injects_matlab_script(qtbot, tmp_path: Path, monkeypatch) -> None:
    window = MatlabHelperMainWindow()
    qtbot.addWidget(window)

    service = StubWindowService()
    monitor = StubKeyboardMonitor()
    injector = StubInjector(service.focus_window)

    script = tmp_path / "demo.m"
    script.write_text("disp('hello world');\n")

    errors: list[str] = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "critical", lambda *args: errors.append(args[2]))

    MainController(
        window,
        window_service=service,
        injector=injector,
        keyboard_monitor=monitor,
        file_dialog=lambda parent: (str(script), ""),
    )

    window.select_file_requested.emit()
    file_label = window.findChild(QtWidgets.QLabel, "FileLabel")
    assert file_label is not None and "demo.m" in file_label.text()

    listener_button = window.findChild(QtWidgets.QPushButton, "ListenerButton")
    assert listener_button is not None
    qtbot.mouseClick(listener_button, QtCore.Qt.MouseButton.LeftButton)

    monitor.fire("space")
    monitor.fire("tab")

    assert injector.calls
    injected_window, first_chunk, tempo, source_inputs = injector.calls[0]
    assert injected_window == service.window
    assert first_chunk == "d"
    assert tempo is None or tempo >= 0
    assert source_inputs == ("space",)
    assert service.focus_calls == len(injector.calls)
    assert errors == []
