"""PyQt6 main window for the MatlabHelper application."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

from PyQt6 import QtCore, QtGui, QtWidgets

from src.config.constants import APP_NAME, ICON_DIR, MAX_PREVIEW_CHARS
from src.models.matlab_window import MatlabWindowInfo
from src.models.typing_settings import TypingSettings


class _IconRegistry:
    def __init__(self) -> None:
        self._cache: dict[str, QtGui.QIcon] = {}

    def get(self, name: str) -> QtGui.QIcon:
        if name not in self._cache:
            icon_path = ICON_DIR / f"{name}.svg"
            if icon_path.exists():
                self._cache[name] = QtGui.QIcon(str(icon_path))
            else:
                self._cache[name] = QtWidgets.QApplication.style().standardIcon(
                    QtWidgets.QStyle.StandardPixmap.SP_FileIcon
                )
        return self._cache[name]

    def pixmap(self, name: str, size: Tuple[int, int] = (24, 24)) -> QtGui.QPixmap:
        icon = self.get(name)
        width, height = size
        pixmap = icon.pixmap(width, height)
        if pixmap.isNull():
            fallback = QtWidgets.QApplication.style().standardIcon(
                QtWidgets.QStyle.StandardPixmap.SP_FileIcon
            )
            return fallback.pixmap(width, height)
        return pixmap


class _TitleBar(QtWidgets.QWidget):
    close_requested = QtCore.pyqtSignal()
    maximize_requested = QtCore.pyqtSignal()
    restore_requested = QtCore.pyqtSignal()

    def __init__(
        self,
        title: str,
        icons: _IconRegistry,
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("TitleBar")
        self._icons = icons
        self._maximized = False
        self._drag_active = False
        self._drag_offset = QtCore.QPoint()

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(12, 6, 12, 6)
        layout.setSpacing(10)

        style = QtWidgets.QApplication.style()

        self._resize_btn = QtWidgets.QToolButton()
        self._resize_btn.setObjectName("TitleButton")
        self._resize_btn.setIcon(
            style.standardIcon(QtWidgets.QStyle.StandardPixmap.SP_TitleBarMaxButton)
        )
        self._resize_btn.setToolTip("Maximize")
        self._resize_btn.clicked.connect(self._on_resize_clicked)
        layout.addWidget(self._resize_btn)

        self._close_btn = QtWidgets.QToolButton()
        self._close_btn.setObjectName("TitleButtonClose")
        self._close_btn.setIcon(
            style.standardIcon(QtWidgets.QStyle.StandardPixmap.SP_TitleBarCloseButton)
        )
        self._close_btn.setToolTip("Close")
        self._close_btn.clicked.connect(self.close_requested.emit)
        layout.addWidget(self._close_btn)

        self._title_label = QtWidgets.QLabel(title)
        self._title_label.setObjectName("TitleLabel")
        self._title_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._title_label.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Preferred,
        )
        layout.addWidget(self._title_label)

        self._app_icon = QtWidgets.QLabel()
        self._app_icon.setObjectName("TitleIcon")
        self._app_icon.setPixmap(self._icons.pixmap("app", (26, 26)))
        layout.addWidget(self._app_icon)

    # ------------------------------------------------------------------
    # API
    # ------------------------------------------------------------------

    def set_title(self, title: str) -> None:
        self._title_label.setText(title)

    def set_app_icon(self, name: str, size: Tuple[int, int] = (26, 26)) -> None:
        self._app_icon.setPixmap(self._icons.pixmap(name, size))

    def set_maximized(self, maximized: bool) -> None:
        self._maximized = maximized
        style = QtWidgets.QApplication.style()
        icon_id = (
            QtWidgets.QStyle.StandardPixmap.SP_TitleBarNormalButton
            if maximized
            else QtWidgets.QStyle.StandardPixmap.SP_TitleBarMaxButton
        )
        tooltip = "Restore" if maximized else "Maximize"
        self._resize_btn.setIcon(style.standardIcon(icon_id))
        self._resize_btn.setToolTip(tooltip)

    # ------------------------------------------------------------------
    # Interaction
    # ------------------------------------------------------------------

    def _on_resize_clicked(self) -> None:
        if self._maximized:
            self.restore_requested.emit()
        else:
            self.maximize_requested.emit()

    def mouseDoubleClickEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._on_resize_clicked()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            child = self.childAt(event.position().toPoint())
            if child in (self._close_btn, self._resize_btn):
                super().mousePressEvent(event)
                return
            window = self.window()
            if isinstance(window, QtWidgets.QWidget) and not window.isMaximized():
                self._drag_active = True
                self._drag_offset = (
                    event.globalPosition().toPoint() - window.frameGeometry().topLeft()
                )
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        window = self.window()
        if (
            self._drag_active
            and isinstance(window, QtWidgets.QWidget)
            and not window.isMaximized()
            and event.buttons() & QtCore.Qt.MouseButton.LeftButton
        ):
            window.move(event.globalPosition().toPoint() - self._drag_offset)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._drag_active = False
        super().mouseReleaseEvent(event)


class _WindowSelectorCard(QtWidgets.QFrame):
    refresh_requested = QtCore.pyqtSignal()
    use_active_requested = QtCore.pyqtSignal()

    def __init__(self, icons: _IconRegistry) -> None:
        super().__init__()
        self.setObjectName("SelectorCard")
        self._icons = icons
        self._windows: dict[int, MatlabWindowInfo] = {}

        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 16, 20, 20)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Target MATLAB Window")
        title.setObjectName("CardTitle")
        header.addWidget(title)
        header.addStretch()

        self._refresh_btn = QtWidgets.QToolButton()
        self._refresh_btn.setIcon(self._icons.get("refresh"))
        self._refresh_btn.setToolTip("Refresh MATLAB windows")
        self._refresh_btn.clicked.connect(self.refresh_requested.emit)
        header.addWidget(self._refresh_btn)

        self._active_btn = QtWidgets.QToolButton()
        self._active_btn.setIcon(self._icons.get("target"))
        self._active_btn.setToolTip("Use currently active MATLAB editor")
        self._active_btn.clicked.connect(self.use_active_requested.emit)
        header.addWidget(self._active_btn)

        layout.addLayout(header)

        self._combo = QtWidgets.QComboBox()
        self._combo.setObjectName("WindowCombo")
        self._combo.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Fixed
        )
        self._combo.currentIndexChanged.connect(self._update_status)
        layout.addWidget(self._combo)

        self._status = QtWidgets.QLabel("No window selected")
        self._status.setObjectName("ComboHint")
        layout.addWidget(self._status)

    def set_windows(self, windows: Iterable[MatlabWindowInfo]) -> None:
        self._windows = {window.handle: window for window in windows}
        self._combo.blockSignals(True)
        self._combo.clear()
        for window in windows:
            self._combo.addItem(window.title, window.handle)
        self._combo.blockSignals(False)
        if self._combo.count() == 0:
            self._combo.addItem("No MATLAB editors found", -1)
            item = self._combo.model().item(0)
            if item is not None:
                item.setEnabled(False)
            self._status.setText("Open a MATLAB editor and refresh.")
        else:
            self._combo.setCurrentIndex(0)
            self._update_status()

    def selected_window(self) -> Optional[MatlabWindowInfo]:
        handle = self._combo.currentData()
        if isinstance(handle, int) and handle in self._windows:
            return self._windows[handle]
        return None

    def set_selected_window(self, window: Optional[MatlabWindowInfo]) -> None:
        if window is None:
            self._combo.setCurrentIndex(-1)
            self._status.setText("No window selected")
            return
        if window.handle not in self._windows:
            self._windows[window.handle] = window
            self._combo.addItem(window.title, window.handle)
        index = self._combo.findData(window.handle)
        self._combo.setCurrentIndex(index)
        self._update_status()

    def _update_status(self) -> None:
        window = self.selected_window()
        if window is None:
            self._status.setText("No window selected")
        else:
            self._status.setText(f"Sending keystrokes to: {window.title}")


@dataclass
class _ProgressSnapshot:
    typed: int
    total: int
    preview: str


class MatlabHelperMainWindow(QtWidgets.QMainWindow):
    """Primary window featuring modular cards, modern styling, and status readouts."""

    select_file_requested = QtCore.pyqtSignal()
    refresh_window_requested = QtCore.pyqtSignal()
    active_window_requested = QtCore.pyqtSignal()
    listener_toggled = QtCore.pyqtSignal(bool)
    settings_changed = QtCore.pyqtSignal(TypingSettings)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowFlags(
            QtCore.Qt.WindowType.FramelessWindowHint
            | QtCore.Qt.WindowType.Window
            | QtCore.Qt.WindowType.WindowSystemMenuHint
            | QtCore.Qt.WindowType.WindowMinimizeButtonHint
        )
        self.setWindowTitle(APP_NAME)
        self.setMinimumSize(980, 600)
        self._icons = _IconRegistry()
        self.setWindowIcon(self._icons.get("app"))
        self._selected_file = None
        self._progress = _ProgressSnapshot(typed=0, total=1, preview="")
        self._title_bar = None
        self._tab_widget = None
        self._log = None
        self._settings_controls = {}
        self._settings_updating = False
        self._current_settings = TypingSettings.from_defaults().clamped()
        self._build_ui()
        self._apply_styling()

    # ------------------------------------------------------------------
    # UI wiring
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        central.setObjectName("MainContainer")
        outer_layout = QtWidgets.QVBoxLayout(central)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        self._title_bar = _TitleBar(APP_NAME, self._icons, self)
        self._title_bar.set_title(APP_NAME)
        self._title_bar.close_requested.connect(self.close)
        self._title_bar.maximize_requested.connect(self._maximize_window)
        self._title_bar.restore_requested.connect(self._restore_window)
        outer_layout.addWidget(self._title_bar)

        body = QtWidgets.QWidget()
        body.setObjectName("BodyContainer")
        body_layout = QtWidgets.QVBoxLayout(body)
        body_layout.setContentsMargins(30, 24, 30, 30)
        body_layout.setSpacing(24)

        hero = self._build_hero_panel()
        body_layout.addWidget(hero)

        content_layout = QtWidgets.QHBoxLayout()
        content_layout.setSpacing(24)
        body_layout.addLayout(content_layout)

        left_column = QtWidgets.QVBoxLayout()
        left_column.setSpacing(18)

        left_column.addWidget(self._build_file_card())

        self._window_selector = _WindowSelectorCard(self._icons)
        self._window_selector.refresh_requested.connect(self.refresh_window_requested.emit)
        self._window_selector.use_active_requested.connect(self.active_window_requested.emit)
        left_column.addWidget(self._window_selector)

        left_column.addWidget(self._build_listener_card())
        left_column.addWidget(self._build_progress_card())
        left_column.addStretch()

        content_layout.addLayout(left_column, stretch=3)

        right_panel = self._build_tab_widget()
        content_layout.addWidget(right_panel, stretch=4)

        outer_layout.addWidget(body)

        self.setCentralWidget(central)
        self.statusBar().showMessage("Select a MATLAB script and target window to begin.")
        if self._title_bar:
            self._title_bar.set_maximized(self.isMaximized())

    def _build_hero_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        panel.setObjectName("HeroPanel")
        layout = QtWidgets.QHBoxLayout(panel)
        layout.setContentsMargins(28, 24, 28, 24)

        title = QtWidgets.QLabel("MatlabHelper")
        title.setObjectName("HeroTitle")
        subtitle = QtWidgets.QLabel(
            "Design captivating MATLAB demos: choose a script, pick your editor, and reveal it keystroke by keystroke."
        )
        subtitle.setObjectName("HeroSubtitle")
        subtitle.setWordWrap(True)

        layout.addWidget(title, alignment=QtCore.Qt.AlignmentFlag.AlignTop)
        layout.addWidget(subtitle)
        layout.addStretch()
        return panel

    def _build_file_card(self) -> QtWidgets.QFrame:
        card = QtWidgets.QFrame()
        card.setObjectName("Card")
        layout = QtWidgets.QVBoxLayout(card)
        layout.setContentsMargins(20, 16, 20, 20)
        layout.setSpacing(12)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("MATLAB Source File")
        title.setObjectName("CardTitle")
        header.addWidget(title)
        header.addStretch()

        self._select_file_btn = QtWidgets.QPushButton("Select Script")
        self._select_file_btn.setIcon(self._icons.get("file"))
        self._select_file_btn.clicked.connect(self.select_file_requested.emit)
        header.addWidget(self._select_file_btn)

        layout.addLayout(header)

        self._file_label = QtWidgets.QLabel("No file selected")
        self._file_label.setObjectName("FileLabel")
        self._file_label.setWordWrap(True)
        layout.addWidget(self._file_label)

        return card

    def _build_listener_card(self) -> QtWidgets.QFrame:
        card = QtWidgets.QFrame()
        card.setObjectName("Card")
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(20, 16, 20, 20)
        layout.setSpacing(16)

        text_layout = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("Keystroke Listener")
        title.setObjectName("CardTitle")
        text_layout.addWidget(title)

        self._listener_status = QtWidgets.QLabel("Idle – press start to capture keys")
        self._listener_status.setObjectName("StatusLabel")
        self._listener_status.setWordWrap(True)
        text_layout.addWidget(self._listener_status)

        layout.addLayout(text_layout)

        self._listener_button = QtWidgets.QPushButton("Start Listening")
        self._listener_button.setObjectName("ListenerButton")
        self._listener_button.setCheckable(True)
        self._listener_button.setIcon(self._icons.get("listen"))
        self._listener_button.clicked.connect(self._on_listener_clicked)
        layout.addWidget(self._listener_button, alignment=QtCore.Qt.AlignmentFlag.AlignRight)

        return card

    def _build_progress_card(self) -> QtWidgets.QFrame:
        card = QtWidgets.QFrame()
        card.setObjectName("Card")
        layout = QtWidgets.QVBoxLayout(card)
        layout.setContentsMargins(20, 16, 20, 20)
        layout.setSpacing(12)

        header = QtWidgets.QLabel("Typing Progress")
        header.setObjectName("CardTitle")
        layout.addWidget(header)

        self._progress_bar = QtWidgets.QProgressBar()
        self._progress_bar.setRange(0, 1)
        self._progress_bar.setValue(0)
        self._progress_bar.setTextVisible(False)
        layout.addWidget(self._progress_bar)

        self._preview_label = QtWidgets.QLabel("Preview will appear here once typing begins.")
        self._preview_label.setObjectName("PreviewLabel")
        self._preview_label.setWordWrap(True)
        layout.addWidget(self._preview_label)

        return card

    def _build_tab_widget(self) -> QtWidgets.QTabWidget:
        tabs = QtWidgets.QTabWidget()
        tabs.setObjectName("SettingsTabs")
        tabs.setTabPosition(QtWidgets.QTabWidget.TabPosition.North)
        tabs.setDocumentMode(True)
        tabs.setElideMode(QtCore.Qt.TextElideMode.ElideRight)
        tabs.setMovable(False)

        log_container = QtWidgets.QWidget()
        log_container.setObjectName("LogTab")
        log_layout = QtWidgets.QVBoxLayout(log_container)
        log_layout.setContentsMargins(16, 16, 16, 16)
        log_layout.setSpacing(12)

        self._log = QtWidgets.QPlainTextEdit()
        self._log.setObjectName("LogConsole")
        self._log.setReadOnly(True)
        log_layout.addWidget(self._log)

        tabs.addTab(log_container, "Activity Log")

        settings_tab = self._build_settings_tab()
        tabs.addTab(settings_tab, "Typing Settings")

        self._tab_widget = tabs
        self._sync_settings_to_controls(self._current_settings)

        return tabs

    def _build_settings_tab(self) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        container.setObjectName("SettingsTab")
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(18)

        description = QtWidgets.QLabel(
            "Fine-tune how the script types into MATLAB, including pacing and typo realism."
        )
        description.setObjectName("SettingsHint")
        description.setWordWrap(True)
        layout.addWidget(description)

        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(
            QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter
        )
        form.setFormAlignment(QtCore.Qt.AlignmentFlag.AlignTop)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        layout.addLayout(form)

        focus_delay = self._create_double_spinbox(0.0, 2.0, 0.01, decimals=2)
        form.addRow("Focus delay (s)", focus_delay)
        self._register_settings_control("focus_delay", focus_delay)

        min_char_delay = self._create_double_spinbox(0.0, 0.5, 0.005, decimals=3)
        form.addRow("Minimum char delay (s)", min_char_delay)
        self._register_settings_control("min_char_delay", min_char_delay)

        max_char_delay = self._create_double_spinbox(0.0, 0.8, 0.005, decimals=3)
        form.addRow("Maximum char delay (s)", max_char_delay)
        self._register_settings_control("max_char_delay", max_char_delay)

        tempo_reference = self._create_double_spinbox(0.05, 2.0, 0.01, decimals=2)
        form.addRow("Tempo reference (s)", tempo_reference)
        self._register_settings_control("tempo_reference_seconds", tempo_reference)

        tempo_min = self._create_double_spinbox(0.0, 1.0, 0.05, decimals=2)
        form.addRow("Tempo min multiplier", tempo_min)
        self._register_settings_control("tempo_min_factor", tempo_min)

        tempo_max = self._create_double_spinbox(0.1, 3.0, 0.05, decimals=2)
        form.addRow("Tempo max multiplier", tempo_max)
        self._register_settings_control("tempo_max_factor", tempo_max)

        error_probability = self._create_double_spinbox(0.0, 1.0, 0.01, decimals=2)
        form.addRow("Typo chance", error_probability)
        self._register_settings_control("error_probability", error_probability)

        error_min = self._create_int_spinbox(0, 10, 1)
        form.addRow("Min typo burst", error_min)
        self._register_settings_control("error_min_burst", error_min)

        error_max = self._create_int_spinbox(1, 12, 1)
        form.addRow("Max typo burst", error_max)
        self._register_settings_control("error_max_burst", error_max)

        layout.addStretch()
        return container

    def _create_double_spinbox(
        self, minimum: float, maximum: float, step: float, *, decimals: int = 2
    ) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setKeyboardTracking(False)
        spin.valueChanged.connect(self._handle_settings_value_changed)
        return spin

    def _create_int_spinbox(self, minimum: int, maximum: int, step: int = 1) -> QtWidgets.QSpinBox:
        spin = QtWidgets.QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setKeyboardTracking(False)
        spin.valueChanged.connect(self._handle_settings_value_changed)
        return spin

    def _register_settings_control(self, key: str, control: QtWidgets.QWidget) -> None:
        self._settings_controls[key] = control

    def _maximize_window(self) -> None:
        self.showMaximized()
        if self._title_bar:
            self._title_bar.set_maximized(True)

    def _restore_window(self) -> None:
        self.showNormal()
        if self._title_bar:
            self._title_bar.set_maximized(False)

    # ------------------------------------------------------------------
    # Public API for the controller
    # ------------------------------------------------------------------

    def set_selected_file(self, file_path: Path) -> None:
        self._selected_file = file_path
        self._file_label.setText(str(file_path))
        self.log_message(f"Selected MATLAB file: {file_path}")

    def clear_selected_file(self) -> None:
        self._selected_file = None
        self._file_label.setText("No file selected")

    def set_window_candidates(self, windows: Iterable[MatlabWindowInfo]) -> None:
        self._window_selector.set_windows(windows)

    def set_selected_window(self, window: Optional[MatlabWindowInfo]) -> None:
        self._window_selector.set_selected_window(window)

    def selected_window(self) -> Optional[MatlabWindowInfo]:
        return self._window_selector.selected_window()

    def set_listener_state(self, active: bool) -> None:
        self._listener_button.setChecked(active)
        self._listener_button.setText("Stop Listening" if active else "Start Listening")
        status_text = (
            "Listening – random keys will reveal the script"
            if active
            else "Idle – press start to capture keys"
        )
        self._listener_status.setText(status_text)

    def update_progress(self, typed: int, total: int, upcoming: str) -> None:
        total = max(total, 1)
        typed = max(0, min(typed, total))
        self._progress = _ProgressSnapshot(typed, total, upcoming)
        self._progress_bar.setRange(0, total)
        self._progress_bar.setValue(typed)
        if upcoming:
            preview = upcoming[:MAX_PREVIEW_CHARS]
            self._preview_label.setText(f"Next characters: {preview}")
        elif typed >= total:
            self._preview_label.setText("All characters delivered.")
        else:
            self._preview_label.setText("Ready for the next keystroke.")

    # ------------------------------------------------------------------
    # Settings management
    # ------------------------------------------------------------------

    def set_settings(self, settings: TypingSettings) -> None:
        self._current_settings = settings.clamped()
        self._sync_settings_to_controls(self._current_settings)

    def _sync_settings_to_controls(self, settings: TypingSettings) -> None:
        if not self._settings_controls:
            return
        self._settings_updating = True
        try:
            self._set_spinbox_value("focus_delay", settings.focus_delay)
            self._set_spinbox_value("min_char_delay", settings.min_char_delay)
            self._set_spinbox_value("max_char_delay", settings.max_char_delay)
            self._set_spinbox_value("tempo_reference_seconds", settings.tempo_reference_seconds)
            self._set_spinbox_value("tempo_min_factor", settings.tempo_min_factor)
            self._set_spinbox_value("tempo_max_factor", settings.tempo_max_factor)
            self._set_spinbox_value("error_probability", settings.error_probability)
            self._set_spinbox_value("error_min_burst", settings.error_min_burst)
            self._set_spinbox_value("error_max_burst", settings.error_max_burst)
        finally:
            self._settings_updating = False

    def _set_spinbox_value(self, key: str, value: float | int) -> None:
        control = self._settings_controls.get(key)
        if control is None:
            return
        if isinstance(control, QtWidgets.QDoubleSpinBox):
            control.setValue(float(value))
        elif isinstance(control, QtWidgets.QSpinBox):
            control.setValue(int(value))

    def _get_control_value(self, key: str, fallback: float | int) -> float | int:
        control = self._settings_controls.get(key)
        if control is None:
            return fallback
        value_method = getattr(control, "value", None)
        if callable(value_method):
            return value_method()
        return fallback

    def _gather_settings_from_controls(self) -> TypingSettings:
        return TypingSettings(
            focus_delay=float(
                self._get_control_value("focus_delay", self._current_settings.focus_delay)
            ),
            min_char_delay=float(
                self._get_control_value("min_char_delay", self._current_settings.min_char_delay)
            ),
            max_char_delay=float(
                self._get_control_value("max_char_delay", self._current_settings.max_char_delay)
            ),
            tempo_reference_seconds=float(
                self._get_control_value(
                    "tempo_reference_seconds", self._current_settings.tempo_reference_seconds
                )
            ),
            tempo_min_factor=float(
                self._get_control_value("tempo_min_factor", self._current_settings.tempo_min_factor)
            ),
            tempo_max_factor=float(
                self._get_control_value("tempo_max_factor", self._current_settings.tempo_max_factor)
            ),
            error_probability=float(
                self._get_control_value(
                    "error_probability", self._current_settings.error_probability
                )
            ),
            error_min_burst=int(
                self._get_control_value("error_min_burst", self._current_settings.error_min_burst)
            ),
            error_max_burst=int(
                self._get_control_value("error_max_burst", self._current_settings.error_max_burst)
            ),
        ).clamped()

    def _handle_settings_value_changed(self, _value: object) -> None:
        if self._settings_updating:
            return
        updated = self._gather_settings_from_controls()
        self._current_settings = updated
        self._sync_settings_to_controls(updated)
        self.settings_changed.emit(updated)

    def log_message(self, message: str) -> None:
        if self._log is None:
            return
        self._log.appendPlainText(message)
        self._log.ensureCursorVisible()

    def _apply_styling(self) -> None:
        palette = self.palette()
        palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor("#0E1117"))
        palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor("#111827"))
        palette.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor("#E6EDF3"))
        palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtGui.QColor("#0E1117"))
        self.setPalette(palette)

        self.setStyleSheet(
            """
            QWidget#MainContainer {
                background-color: #060913;
            }
            QWidget#BodyContainer {
                background-color: #0E1117;
            }
            QWidget#TitleBar {
                background-color: #070B14;
                border-bottom: 1px solid rgba(56, 68, 104, 0.6);
            }
            QLabel#TitleLabel {
                font-size: 14px;
                font-weight: 600;
                letter-spacing: 0.6px;
                color: #E6EDF3;
            }
            QLabel#TitleIcon {
                padding-right: 6px;
            }
            QToolButton#TitleButton, QToolButton#TitleButtonClose {
                background-color: transparent;
                border: none;
                padding: 6px;
                border-radius: 10px;
            }
            QToolButton#TitleButton:hover {
                background-color: rgba(34, 211, 238, 0.2);
            }
            QToolButton#TitleButton:pressed {
                background-color: rgba(34, 211, 238, 0.35);
            }
            QToolButton#TitleButtonClose:hover {
                background-color: rgba(248, 113, 113, 0.85);
            }
            QToolButton#TitleButtonClose:pressed {
                background-color: rgba(248, 113, 113, 0.65);
            }
            QWidget#HeroPanel {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                              stop:0 #1F6FEB, stop:1 #0B1F4F);
                border-radius: 18px;
                color: #FFFFFF;
            }
            QLabel#HeroTitle {
                font-size: 28px;
                font-weight: 700;
                color: #FFFFFF;
            }
            QLabel#HeroSubtitle {
                font-size: 16px;
                color: rgba(255, 255, 255, 0.86);
                max-width: 460px;
            }
            QFrame#Card, QFrame#SelectorCard {
                background-color: rgba(17, 24, 39, 0.88);
                border: 1px solid rgba(46, 58, 89, 0.7);
                border-radius: 16px;
            }
            QLabel#CardTitle {
                font-size: 16px;
                font-weight: 600;
                color: #E6EDF3;
            }
            QLabel#FileLabel, QLabel#StatusLabel, QLabel#PreviewLabel, QLabel#ComboHint {
                color: rgba(230, 237, 243, 0.9);
                font-size: 14px;
            }
            QPlainTextEdit#LogConsole {
                background-color: rgba(10, 12, 18, 0.9);
                border: 1px solid rgba(46, 58, 89, 0.7);
                border-radius: 16px;
                padding: 16px;
                font-family: "Cascadia Code", "Fira Code", monospace;
                font-size: 13px;
                color: #E6EDF3;
            }
            QPushButton#ListenerButton {
                padding: 12px 24px;
                border-radius: 22px;
                font-weight: 600;
                background-color: #22D3EE;
                color: #0F172A;
            }
            QPushButton#ListenerButton:checked {
                background-color: #F97316;
                color: #0F172A;
            }
            QPushButton {
                padding: 10px 18px;
                border-radius: 14px;
                font-weight: 600;
                color: #0F172A;
                background-color: #38BDF8;
            }
            QPushButton:hover {
                background-color: #0EA5E9;
            }
            QPushButton:disabled {
                background-color: rgba(148, 163, 184, 0.4);
                color: rgba(15, 23, 42, 0.6);
            }
            QToolButton {
                background-color: transparent;
                border: none;
                padding: 6px;
            }
            QToolButton:hover {
                background-color: rgba(56, 189, 248, 0.15);
                border-radius: 12px;
            }
            QComboBox#WindowCombo {
                padding: 8px 12px;
                border-radius: 12px;
                background-color: rgba(15, 23, 42, 0.8);
                border: 1px solid rgba(148, 163, 184, 0.4);
                color: #E6EDF3;
            }
            QComboBox#WindowCombo::drop-down {
                border: none;
            }
            QProgressBar {
                border-radius: 12px;
                background-color: rgba(15, 23, 42, 0.7);
                border: 1px solid rgba(148, 163, 184, 0.4);
                height: 18px;
            }
            QProgressBar::chunk {
                border-radius: 12px;
                background-color: #22D3EE;
            }
            QTabWidget#SettingsTabs::pane {
                border-radius: 16px;
                border: 1px solid rgba(46, 58, 89, 0.7);
                background-color: rgba(10, 12, 18, 0.9);
                padding: 8px;
            }
            QTabWidget#SettingsTabs QTabBar::tab {
                background-color: rgba(15, 23, 42, 0.85);
                color: #E6EDF3;
                padding: 10px 18px;
                border-top-left-radius: 12px;
                border-top-right-radius: 12px;
                margin-right: 6px;
            }
            QTabWidget#SettingsTabs QTabBar::tab:selected {
                background-color: #1F2937;
            }
            QWidget#SettingsTab {
                background-color: transparent;
            }
            QLabel#SettingsHint {
                color: rgba(226, 232, 240, 0.78);
                font-size: 13px;
            }
        """
        )

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_listener_clicked(self, checked: bool) -> None:
        self.listener_toggled.emit(checked)

    def changeEvent(self, event: QtCore.QEvent) -> None:
        super().changeEvent(event)
        if event.type() == QtCore.QEvent.Type.WindowStateChange and self._title_bar:
            self._title_bar.set_maximized(self.isMaximized())

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # pragma: no cover - Qt lifecycle
        self.listener_toggled.emit(False)
        super().closeEvent(event)
