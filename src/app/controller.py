"""Main application controller coordinating UI and services."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Callable, Optional, Tuple

from PyQt6 import QtCore, QtWidgets

from src.config.constants import MAX_PREVIEW_CHARS
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
        self._script_content: str = ""
        self._typed_index: int = 0
        self._last_key_timestamp: Optional[float] = None

        self._connect_signals()
        self._refresh_matlab_windows()

    # ------------------------------------------------------------------
    # Signal wiring
    # ------------------------------------------------------------------

    def _connect_signals(self) -> None:
        self._window.select_file_requested.connect(self._handle_file_selection)
        self._window.refresh_window_requested.connect(self._refresh_matlab_windows)
        self._window.active_window_requested.connect(self._select_active_window)
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
        self._load_script_content()

    # ------------------------------------------------------------------
    # MATLAB window detection
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _refresh_matlab_windows(self) -> None:
        try:
            windows = self._window_service.list_matlab_windows()
        except MatlabWindowDetectionError as exc:
            self._matlab_window = None
            self._window.set_window_candidates([])
            self._window.log_message(str(exc))
            return

        self._window.set_window_candidates(windows)
        if self._matlab_window is not None:
            self._window.set_selected_window(self._matlab_window)
        elif windows:
            self._matlab_window = windows[0]
            self._window.set_selected_window(self._matlab_window)

    @QtCore.pyqtSlot()
    def _select_active_window(self) -> None:
        try:
            active = self._window_service.get_active_matlab_window()
        except MatlabWindowDetectionError as exc:
            self._window.log_message(str(exc))
            return
        if active is None:
            self._window.log_message("No active MATLAB editor detected.")
            return
        self._matlab_window = active
        self._window.set_selected_window(active)

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
            if not self._script_content:
                self._show_error("Empty Script", "Selected MATLAB file is empty.")
                self._window.set_listener_state(False)
                return
            selected = self._window.selected_window()
            if selected is None:
                self._show_error(
                    "MATLAB Editor Required", "Select a MATLAB editor window to receive keystrokes."
                )
                self._window.set_listener_state(False)
                return
            self._matlab_window = selected
            if self._typed_index >= len(self._script_content):
                self._window.log_message(
                    "All characters have already been injected. Reset by reloading the file."
                )
                self._window.set_listener_state(False)
                return
            self._last_key_timestamp = None
            try:
                self._keyboard_monitor.start()
                self._window.log_message(
                    "Listener armed. Press any key to reveal the next character."
                )
            except Exception as exc:  # pragma: no cover - keyboard lib failure
                self._show_error("Keyboard Hook Error", str(exc))
                self._window.set_listener_state(False)
        else:
            try:
                self._keyboard_monitor.stop()
            except Exception as exc:  # pragma: no cover
                self._window.log_message(f"Failed to stop listener cleanly: {exc}")
            finally:
                self._last_key_timestamp = None

    @QtCore.pyqtSlot(str)
    def _handle_global_key(self, key_name: str) -> None:
        if not self._selected_file or not self._matlab_window:
            self._window.log_message("Listener triggered without prerequisites.")
            return

        now = time.perf_counter()
        tempo_hint = None
        if self._last_key_timestamp is not None:
            tempo_hint = now - self._last_key_timestamp

        chunk = self._next_chunk()
        if not chunk:
            self._window.log_message("All characters injected; ignoring extra key press.")
            self._window.set_listener_state(False)
            self._last_key_timestamp = now
            return

        try:
            self._injector.type_text(self._matlab_window, chunk, tempo_hint=tempo_hint)
            self._window.log_message(
                f"Key '{key_name}' revealed {len(chunk)} character{'s' if len(chunk) != 1 else ''}."
            )
        except (MatlabFileError, MatlabInjectionError) as exc:
            self._show_error("Injection Failed", str(exc))
        finally:
            self._update_progress()
            if self._typed_index >= len(self._script_content):
                self._window.log_message("Completed MATLAB script playback.")
                self._window.set_listener_state(False)
        self._last_key_timestamp = now

    # ------------------------------------------------------------------
    # Typing progression
    # ------------------------------------------------------------------

    def _load_script_content(self) -> None:
        if not self._selected_file:
            self._script_content = ""
            self._typed_index = 0
            self._update_progress()
            return
        try:
            raw = read_matlab_source(self._selected_file)
        except MatlabFileError as exc:
            self._show_error("Read Error", str(exc))
            self._script_content = ""
            self._typed_index = 0
            self._update_progress()
            return
        sanitized = raw.replace("\r\n", "\n").replace("\r", "\n")
        self._script_content = sanitized
        self._typed_index = 0
        self._update_progress()

    def _next_chunk(self) -> str:
        if self._typed_index >= len(self._script_content):
            return ""
        char = self._script_content[self._typed_index]
        self._typed_index += 1
        return char

    def _update_progress(self) -> None:
        upcoming = self._script_content[self._typed_index : self._typed_index + MAX_PREVIEW_CHARS]
        self._window.update_progress(self._typed_index, len(self._script_content), upcoming)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _show_error(self, title: str, message: str) -> None:
        QtWidgets.QMessageBox.critical(self._window, title, message)
        self._window.log_message(f"Error: {message}")
