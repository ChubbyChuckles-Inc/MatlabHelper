"""Main application controller coordinating UI and services."""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional, Tuple

from PyQt6 import QtCore, QtWidgets

from src.models.matlab_window import MatlabWindowInfo
from src.services.file_service import MatlabFileError, ensure_matlab_file, read_matlab_source
from src.services.injector_service import MatlabInjectionError, MatlabInjector
from src.services.keyboard_monitor import KeyboardMonitor
from src.services.window_service import MatlabWindowDetectionError, MatlabWindowService
from src.ui.main_window import MatlabHelperMainWindow


FileDialogFn = Callable[[QtWidgets.QWidget], Tuple[str, str]]


class MainController(QtCore.QObject):
    """Glue between the PyQt UI, keyboard monitor, and MATLAB window services."""

    def __init__(
        self,
        window: MatlabHelperMainWindow,
        *,
        window_service: Optional[MatlabWindowService] = None,
        injector: Optional[MatlabInjector] = None,
        keyboard_monitor: Optional[KeyboardMonitor] = None,
        file_dialog: Optional[FileDialogFn] = None,
    ) -> None:
        super().__init__(window)
        self._window = window
        self._window_service = window_service or MatlabWindowService()
        self._keyboard_monitor = keyboard_monitor or KeyboardMonitor()
        self._injector = injector or MatlabInjector(self._window_service)
        self._file_dialog = file_dialog or self._default_file_dialog

        self._selected_file: Optional[Path] = None
        self._matlab_window: Optional[MatlabWindowInfo] = None

        self._connect_signals()
        self._refresh_matlab_window_status()

    # ------------------------------------------------------------------
    # Signal wiring
    # ------------------------------------------------------------------

    def _connect_signals(self) -> None:
        self._window.select_file_requested.connect(self._handle_file_selection)
        self._window.refresh_window_requested.connect(self._refresh_matlab_window_status)
        self._window.listener_toggled.connect(self._toggle_listener)
        self._keyboard_monitor.key_pressed.connect(self._handle_global_key)
        self._keyboard_monitor.listening_changed.connect(self._window.set_listener_state)

    # ------------------------------------------------------------------
    # File selection
    # ------------------------------------------------------------------

    def _default_file_dialog(self, parent: QtWidgets.QWidget) -> tuple[str, str]:
        return QtWidgets.QFileDialog.getOpenFileName(
            parent,
            "Select MATLAB Script",
            str(Path.home()),
            "MATLAB Scripts (*.m)",
        )

    @QtCore.pyqtSlot()
    def _handle_file_selection(self) -> None:
        file_name, _ = self._file_dialog(self._window)
        if not file_name:
            return

        try:
            path = ensure_matlab_file(Path(file_name))
        except MatlabFileError as exc:
            self._show_error("Invalid MATLAB File", str(exc))
            self._window.clear_selected_file()
            self._selected_file = None
            return

        self._selected_file = path
        self._window.set_selected_file(path)

    # ------------------------------------------------------------------
    # MATLAB window detection
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _refresh_matlab_window_status(self) -> None:
        try:
            self._matlab_window = self._window_service.get_active_matlab_window()
        except MatlabWindowDetectionError as exc:
            self._matlab_window = None
            self._window.set_matlab_window(None)
            self._window.log_message(str(exc))
            return

        title = self._matlab_window.title if self._matlab_window else None
        self._window.set_matlab_window(title)

    # ------------------------------------------------------------------
    # Keyboard listening
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(bool)
    def _toggle_listener(self, requested_state: bool) -> None:
        if requested_state:
            if not self._selected_file:
                self._show_error("MATLAB File Required", "Choose a MATLAB script before listening.")
                self._window.set_listener_state(False)
                return
            try:
                self._matlab_window = self._window_service.get_active_matlab_window()
            except MatlabWindowDetectionError as exc:
                self._show_error("Unsupported Platform", str(exc))
                self._window.set_listener_state(False)
                return
            if not self._matlab_window:
                self._show_error(
                    "MATLAB Editor Not Found",
                    "Focus a MATLAB editor window before starting the listener.",
                )
                self._window.set_listener_state(False)
                return
            try:
                self._keyboard_monitor.start()
                self._window.log_message("Listening for the next random key press...")
            except Exception as exc:  # pragma: no cover - keyboard lib failure
                self._show_error("Keyboard Hook Error", str(exc))
                self._window.set_listener_state(False)
        else:
            try:
                self._keyboard_monitor.stop()
            except Exception as exc:  # pragma: no cover
                self._window.log_message(f"Failed to stop listener cleanly: {exc}")

    @QtCore.pyqtSlot(str)
    def _handle_global_key(self, key_name: str) -> None:
        if not self._selected_file or not self._matlab_window:
            self._window.log_message("Listener triggered without prerequisites.")
            return

        try:
            content = read_matlab_source(self._selected_file)
            self._injector.inject(self._matlab_window, content)
            self._window.log_message(f"Injected MATLAB script after key '{key_name}'.")
        except (MatlabFileError, MatlabInjectionError) as exc:
            self._show_error("Injection Failed", str(exc))
        finally:
            self._window.set_listener_state(False)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _show_error(self, title: str, message: str) -> None:
        QtWidgets.QMessageBox.critical(self._window, title, message)
        self._window.log_message(f"Error: {message}")
