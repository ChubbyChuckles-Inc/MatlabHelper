"""Main application controller coordinating UI and services."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Callable, Optional, Tuple

from PyQt6 import QtCore, QtWidgets

from src.config.constants import MAX_PREVIEW_CHARS
from src.models.app_state import AppState
from src.models.matlab_window import MatlabWindowInfo
from src.models.typing_settings import TypingSettings
from src.services.file_service import MatlabFileError, ensure_matlab_file, read_matlab_source
from src.services.injector_service import MatlabInjectionError, MatlabInjector
from src.services.keyboard_monitor import KeyboardMonitor
from src.services.window_service import MatlabWindowDetectionError, MatlabWindowService
from src.ui.main_window import MatlabHelperMainWindow
from src.utils.app_state_store import AppStateStore


FileDialogFn = Callable[[QtWidgets.QWidget], Tuple[str, str]]

DEFAULT_STATUS_MESSAGE = "Select a MATLAB script and target window to begin."
LISTENER_ARMED_MESSAGE = "Listener armed. Press any key to reveal the next character."
LISTENER_PAUSED_MESSAGE = "Listener paused. Press AltGr to resume."


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
        self._state_store = AppStateStore()
        self._app_state = self._state_store.load()
        self._typing_settings = self._app_state.typing_settings.clamped()
        self._app_state.typing_settings = self._typing_settings
        self._injector = injector or MatlabInjector(
            self._window_service, settings=self._typing_settings
        )
        if hasattr(self._injector, "update_settings"):
            self._injector.update_settings(self._typing_settings)
        self._file_dialog = file_dialog or self._default_file_dialog

        self._selected_file: Optional[Path] = None
        self._matlab_window: Optional[MatlabWindowInfo] = None
        self._script_content: str = ""
        self._typed_index: int = 0
        self._last_key_timestamp: Optional[float] = None
        self._listener_paused = False

        self._state_save_timer = QtCore.QTimer(self)
        self._state_save_timer.setSingleShot(True)
        self._state_save_timer.setInterval(750)
        self._state_save_timer.timeout.connect(self._save_state)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self._save_state)

        self._connect_signals()
        self._restore_ui_state()
        self._refresh_matlab_windows()

    # ------------------------------------------------------------------
    # Signal wiring
    # ------------------------------------------------------------------

    def _connect_signals(self) -> None:
        self._window.select_file_requested.connect(self._handle_file_selection)
        self._window.refresh_window_requested.connect(self._refresh_matlab_windows)
        self._window.active_window_requested.connect(self._select_active_window)
        self._window.listener_toggled.connect(self._toggle_listener)
        self._window.target_window_changed.connect(self._handle_target_window_changed)
        self._window.tab_changed.connect(self._handle_tab_changed)
        settings_changed = getattr(self._window, "settings_changed", None)
        if callable(getattr(settings_changed, "connect", None)):
            settings_changed.connect(self._handle_settings_changed)
        self._keyboard_monitor.key_pressed.connect(self._handle_global_key)
        self._keyboard_monitor.listening_changed.connect(self._on_listening_changed)

    def _restore_ui_state(self) -> None:
        if hasattr(self._window, "set_settings"):
            self._window.set_settings(self._typing_settings)
        self._window.set_tab_index(self._app_state.active_tab)
        self._window.set_log_entries(self._app_state.log_messages)
        status_message = self._app_state.status_message or DEFAULT_STATUS_MESSAGE
        if status_message == LISTENER_ARMED_MESSAGE:
            status_message = DEFAULT_STATUS_MESSAGE
        self._window.set_status_message(status_message)
        if self._app_state.selected_file:
            path = Path(self._app_state.selected_file)
            if path.exists():
                self._selected_file = path
                self._window.set_selected_file(path)
                self._load_script_content()
                if not self._file_matches_state(path):
                    self._typed_index = 0
                else:
                    self._typed_index = max(
                        0, min(self._app_state.typed_index, len(self._script_content))
                    )
                self._update_progress()
            else:
                self._selected_file = None
                self._window.clear_selected_file()
                self._app_state.selected_file = None
                self._app_state.selected_file_mtime = None
                self._app_state.typed_index = 0
        self._app_state.listener_active = False
        self._listener_paused = False
        self._window.set_listener_state(False, paused=False)

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
            self._app_state.selected_file = None
            self._app_state.selected_file_mtime = None
            self._app_state.typed_index = 0
            self._schedule_state_save()
            return

        self._selected_file = path
        self._window.set_selected_file(path)
        self._app_state.selected_file = str(path)
        self._app_state.selected_file_mtime = self._get_file_mtime(path)
        self._app_state.typed_index = 0
        self._log(f"Selected MATLAB file: {path}")
        self._load_script_content()
        self._schedule_state_save()

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
            self._update_target_window_state(None)
            self._log(str(exc))
            return

        self._window.set_window_candidates(windows)
        self._apply_window_selection_from_state(tuple(windows))

    @QtCore.pyqtSlot()
    def _select_active_window(self) -> None:
        try:
            active = self._window_service.get_active_matlab_window()
        except MatlabWindowDetectionError as exc:
            self._log(str(exc))
            return
        if active is None:
            self._log("No active MATLAB editor detected.")
            self._update_target_window_state(None)
            return
        self._matlab_window = active
        self._window.set_selected_window(active)
        self._update_target_window_state(active)

    # ------------------------------------------------------------------
    # Keyboard listening
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(bool)
    def _toggle_listener(self, requested_state: bool) -> None:
        if requested_state:
            if not self._selected_file:
                self._show_error("MATLAB File Required", "Choose a MATLAB script before listening.")
                self._window.set_listener_state(False)
                self._window.set_status_message(DEFAULT_STATUS_MESSAGE)
                return
            if not self._script_content:
                self._show_error("Empty Script", "Selected MATLAB file is empty.")
                self._window.set_listener_state(False)
                self._window.set_status_message(DEFAULT_STATUS_MESSAGE)
                return
            selected = self._window.selected_window()
            if selected is None:
                self._show_error(
                    "MATLAB Editor Required", "Select a MATLAB editor window to receive keystrokes."
                )
                self._window.set_listener_state(False)
                self._window.set_status_message(DEFAULT_STATUS_MESSAGE)
                return
            self._matlab_window = selected
            self._listener_paused = False
            if hasattr(self._window, "set_listener_pause_state"):
                self._window.set_listener_pause_state(False)
            if self._typed_index >= len(self._script_content):
                self._log("All characters have already been injected. Reset by reloading the file.")
                self._window.set_listener_state(False)
                self._window.set_status_message(DEFAULT_STATUS_MESSAGE)
                return
            self._last_key_timestamp = None
            if hasattr(self._injector, "reset_focus_cache"):
                self._injector.reset_focus_cache()
            try:
                self._keyboard_monitor.set_passthrough_enabled(False)
                self._keyboard_monitor.start()
                self._log("Listener armed. Press any key to reveal the next character.")
            except Exception as exc:  # pragma: no cover - keyboard lib failure
                self._show_error("Keyboard Hook Error", str(exc))
                self._window.set_listener_state(False)
        else:
            try:
                self._keyboard_monitor.stop()
            except Exception as exc:  # pragma: no cover
                self._log(f"Failed to stop listener cleanly: {exc}")
            finally:
                self._last_key_timestamp = None
                if hasattr(self._injector, "reset_focus_cache"):
                    self._injector.reset_focus_cache()
                self._listener_paused = False
                if hasattr(self._window, "set_listener_pause_state"):
                    self._window.set_listener_pause_state(False)
                try:
                    self._keyboard_monitor.set_passthrough_enabled(False)
                except Exception:
                    pass

    @QtCore.pyqtSlot(str)
    def _handle_global_key(self, key_name: str) -> None:
        norm_key = key_name.replace(" ", "").replace("_", "").lower()
        if norm_key in {"altgr", "altgraph"}:
            self._toggle_pause()
            return

        if not self._selected_file or not self._matlab_window:
            self._log("Listener triggered without prerequisites.")
            return

        if self._listener_paused:
            return

        now = time.perf_counter()
        tempo_hint = None
        if self._last_key_timestamp is not None:
            tempo_hint = now - self._last_key_timestamp

        injector_pending = bool(getattr(self._injector, "has_pending_work", lambda: False)())
        chunk = self._next_chunk()
        if not chunk and not injector_pending:
            self._log("All characters injected; ignoring extra key press.")
            self._listener_paused = False
            if hasattr(self._window, "set_listener_pause_state"):
                self._window.set_listener_pause_state(False)
            self._window.set_listener_state(False)
            self._last_key_timestamp = now
            return

        try:
            self._injector.type_text(
                self._matlab_window,
                chunk if chunk else "",
                tempo_hint=tempo_hint,
                source_inputs=(key_name,),
            )
            if chunk:
                self._log(
                    f"Key '{key_name}' revealed {len(chunk)} character{'s' if len(chunk) != 1 else ''}."
                )
            else:
                self._log("Key press applied a queued correction.")
        except (MatlabFileError, MatlabInjectionError) as exc:
            self._show_error("Injection Failed", str(exc))
        finally:
            self._update_progress()
            injector_has_remaining = bool(
                getattr(self._injector, "has_pending_work", lambda: False)()
            )
            if self._typed_index >= len(self._script_content) and not injector_has_remaining:
                self._log("Completed MATLAB script playback.")
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
            self._app_state.typed_index = 0
            self._app_state.selected_file_mtime = None
            self._schedule_state_save()
            return
        try:
            raw = read_matlab_source(self._selected_file)
        except MatlabFileError as exc:
            self._show_error("Read Error", str(exc))
            self._script_content = ""
            self._typed_index = 0
            self._update_progress()
            self._app_state.typed_index = 0
            self._schedule_state_save()
            return
        sanitized = raw.replace("\r\n", "\n").replace("\r", "\n")
        self._script_content = sanitized
        self._typed_index = 0
        self._app_state.selected_file_mtime = self._get_file_mtime(self._selected_file)
        self._update_progress()
        self._app_state.typed_index = 0
        self._schedule_state_save()

    def _next_chunk(self) -> str:
        if self._typed_index >= len(self._script_content):
            return ""
        char = self._script_content[self._typed_index]
        self._typed_index += 1
        return char

    def _update_progress(self) -> None:
        upcoming = self._script_content[self._typed_index : self._typed_index + MAX_PREVIEW_CHARS]
        self._window.update_progress(self._typed_index, len(self._script_content), upcoming)
        self._app_state.typed_index = self._typed_index
        self._schedule_state_save()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(object)
    def _handle_settings_changed(self, settings: TypingSettings) -> None:
        self._typing_settings = settings.clamped()
        if hasattr(self._injector, "update_settings"):
            self._injector.update_settings(self._typing_settings)
        self._app_state.typing_settings = self._typing_settings
        self._log("Updated typing settings.")

    def _show_error(self, title: str, message: str) -> None:
        QtWidgets.QMessageBox.critical(self._window, title, message)
        self._log(f"Error: {message}")

    # ------------------------------------------------------------------
    # Persistence helpers
    # ------------------------------------------------------------------

    def apply_initial_state(self) -> None:
        geometry = self._app_state.window_geometry
        if geometry:
            try:
                data = QtCore.QByteArray.fromBase64(geometry.encode("ascii"))
                self._window.restoreGeometry(data)
            except Exception:  # pragma: no cover - malformed geometry payload
                pass
        if self._app_state.window_maximized:
            self._window.setWindowState(
                self._window.windowState() | QtCore.Qt.WindowState.WindowMaximized
            )

    def _handle_target_window_changed(self, window: Optional[MatlabWindowInfo]) -> None:
        self._matlab_window = window
        self._update_target_window_state(window)

    def _handle_tab_changed(self, index: int) -> None:
        self._app_state.active_tab = max(0, index)
        self._schedule_state_save()

    def _on_listening_changed(self, active: bool) -> None:
        if not active:
            self._listener_paused = False
        self._window.set_listener_state(active, paused=self._listener_paused)
        self._app_state.listener_active = active
        if active:
            self._window.set_status_message(LISTENER_ARMED_MESSAGE)
        else:
            self._window.set_status_message(DEFAULT_STATUS_MESSAGE)
        self._schedule_state_save()

    def _toggle_pause(self) -> None:
        if not self._keyboard_monitor.is_listening():
            return
        self._listener_paused = not self._listener_paused
        if hasattr(self._window, "set_listener_pause_state"):
            self._window.set_listener_pause_state(self._listener_paused)
        if self._listener_paused:
            try:
                self._keyboard_monitor.set_passthrough_enabled(True)
            except Exception as exc:  # pragma: no cover - keyboard lib failure
                self._show_error("Keyboard Hook Error", str(exc))
                self._listener_paused = False
                if hasattr(self._window, "set_listener_pause_state"):
                    self._window.set_listener_pause_state(False)
                return
            self._window.set_status_message(LISTENER_PAUSED_MESSAGE)
            self._log("Listener paused. Press AltGr to resume.")
            self._last_key_timestamp = None
        else:
            try:
                self._keyboard_monitor.set_passthrough_enabled(False)
            except Exception as exc:  # pragma: no cover - keyboard lib failure
                self._show_error("Keyboard Hook Error", str(exc))
                self._listener_paused = True
                if hasattr(self._window, "set_listener_pause_state"):
                    self._window.set_listener_pause_state(True)
                return
            self._window.set_status_message(LISTENER_ARMED_MESSAGE)
            self._log("Listener resumed.")
        self._schedule_state_save()

    def _apply_window_selection_from_state(self, windows: Tuple[MatlabWindowInfo, ...]) -> None:
        if not windows:
            self._matlab_window = None
            self._window.set_selected_window(None)
            self._update_target_window_state(None)
            return

        target: Optional[MatlabWindowInfo] = None
        if self._app_state.target_window_handle is not None:
            target = next(
                (
                    window
                    for window in windows
                    if window.handle == self._app_state.target_window_handle
                ),
                None,
            )
        if target is None and self._app_state.target_window_title:
            target = next(
                (
                    window
                    for window in windows
                    if window.title == self._app_state.target_window_title
                ),
                None,
            )
        if target is None and self._matlab_window is not None:
            target = next(
                (window for window in windows if window.handle == self._matlab_window.handle),
                None,
            )
        if target is None:
            target = windows[0]

        self._matlab_window = target
        self._window.set_selected_window(target)
        self._update_target_window_state(target)

    def _update_target_window_state(self, window: Optional[MatlabWindowInfo]) -> None:
        if window is None:
            self._app_state.target_window_handle = None
            self._app_state.target_window_title = None
        else:
            self._app_state.target_window_handle = window.handle
            self._app_state.target_window_title = window.title
        self._schedule_state_save()

    def _schedule_state_save(self) -> None:
        if self._state_save_timer.isActive():
            self._state_save_timer.stop()
        self._state_save_timer.start()

    @QtCore.pyqtSlot()
    def _save_state(self) -> None:
        if self._state_save_timer.isActive():
            self._state_save_timer.stop()
        self._app_state.typing_settings = self._typing_settings
        self._app_state.listener_active = self._keyboard_monitor.is_listening()
        self._app_state.active_tab = self._window.tab_index()
        self._app_state.log_messages = self._window.get_log_entries()
        self._app_state.status_message = self._window.status_message()
        geometry = self._window.saveGeometry()
        self._app_state.window_geometry = bytes(geometry.toBase64()).decode("ascii")
        self._app_state.window_maximized = self._window.isMaximized()
        if self._selected_file is None:
            self._app_state.selected_file = None
            self._app_state.selected_file_mtime = None
        else:
            self._app_state.selected_file = str(self._selected_file)
            self._app_state.selected_file_mtime = self._get_file_mtime(self._selected_file)
        self._state_store.save(self._app_state)

    def _file_matches_state(self, path: Path) -> bool:
        if self._app_state.selected_file_mtime is None:
            return False
        mtime = self._get_file_mtime(path)
        if mtime is None:
            return False
        return abs(mtime - self._app_state.selected_file_mtime) < 1e-6

    @staticmethod
    def _get_file_mtime(path: Path) -> Optional[float]:
        try:
            return path.stat().st_mtime
        except OSError:
            return None

    def _log(self, message: str) -> None:
        self._window.log_message(message)
        self._app_state.update_logs(message)
        self._schedule_state_save()
