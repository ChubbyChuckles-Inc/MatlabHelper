"""PyQt6 main window for the MatlabHelper application."""

from __future__ import annotations

from pathlib import Path

from PyQt6 import QtCore, QtGui, QtWidgets

from src.config.constants import APP_NAME


class MatlabHelperMainWindow(QtWidgets.QMainWindow):
    """Primary window featuring file selection, status panels, and listener controls."""

    select_file_requested = QtCore.pyqtSignal()
    refresh_window_requested = QtCore.pyqtSignal()
    listener_toggled = QtCore.pyqtSignal(bool)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.setMinimumSize(800, 480)
        self._selected_file: Path | None = None
        self._build_ui()
        self._apply_styling()

    # ------------------------------------------------------------------
    # UI wiring
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(18)
        layout.setContentsMargins(24, 24, 24, 24)

        hero = QtWidgets.QLabel(
            "Stream MATLAB scripts into the active editor with a single keystroke."
        )
        hero.setWordWrap(True)
        hero.setObjectName("HeroLabel")
        layout.addWidget(hero)

        file_group = self._build_file_group()
        layout.addWidget(file_group)

        matlab_group = self._build_matlab_group()
        layout.addWidget(matlab_group)

        listener_group = self._build_listener_group()
        layout.addWidget(listener_group)

        self._log = QtWidgets.QTextEdit()
        self._log.setReadOnly(True)
        self._log.setObjectName("LogConsole")
        self._log.setPlaceholderText("Status messages will appear here...")
        layout.addWidget(self._log, stretch=1)

        central.setLayout(layout)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Select a MATLAB script to begin.")

    def _build_file_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("MATLAB Source")
        layout = QtWidgets.QHBoxLayout()

        self._file_label = QtWidgets.QLabel("No file selected")
        self._file_label.setObjectName("FileLabel")
        self._file_label.setWordWrap(True)

        browse_btn = QtWidgets.QPushButton("Select MATLAB Script")
        browse_btn.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_DialogOpenButton)
        )
        browse_btn.clicked.connect(self.select_file_requested.emit)

        layout.addWidget(self._file_label, stretch=1)
        layout.addWidget(browse_btn)
        group.setLayout(layout)
        return group

    def _build_matlab_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("MATLAB Editor")
        layout = QtWidgets.QHBoxLayout()

        self._matlab_label = QtWidgets.QLabel("No MATLAB editor detected")
        self._matlab_label.setObjectName("MatlabLabel")
        self._matlab_label.setWordWrap(True)

        refresh_btn = QtWidgets.QPushButton("Refresh")
        refresh_btn.setIcon(
            self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_BrowserReload)
        )
        refresh_btn.clicked.connect(self.refresh_window_requested.emit)

        layout.addWidget(self._matlab_label, stretch=1)
        layout.addWidget(refresh_btn)
        group.setLayout(layout)
        return group

    def _build_listener_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Keystroke Listener")
        layout = QtWidgets.QHBoxLayout()

        self._listener_button = QtWidgets.QPushButton("Start Listening")
        self._listener_button.setCheckable(True)
        self._listener_button.setObjectName("ListenerButton")
        self._listener_button.clicked.connect(self._on_listener_clicked)

        self._listener_status = QtWidgets.QLabel("Idle")
        self._listener_status.setObjectName("ListenerStatus")

        layout.addWidget(self._listener_button)
        layout.addWidget(self._listener_status, alignment=QtCore.Qt.AlignmentFlag.AlignRight)
        group.setLayout(layout)
        return group

    def _apply_styling(self) -> None:
        palette = self.palette()
        palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor("#101820"))
        palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor("#F2AA4C"))
        palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor("#1C1C1C"))
        palette.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor("#F8F8F8"))
        palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor("#F2AA4C"))
        palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtGui.QColor("#101820"))
        self.setPalette(palette)
        self.setStyleSheet(
            """
            QLabel#HeroLabel {
                font-size: 22px;
                font-weight: 600;
                color: #F2AA4C;
            }
            QLabel#FileLabel, QLabel#MatlabLabel, QLabel#ListenerStatus {
                font-size: 14px;
                color: #F8F8F8;
            }
            QTextEdit#LogConsole {
                background-color: #141414;
                color: #EEEEEE;
                border-radius: 8px;
                padding: 12px;
            }
            QGroupBox {
                font-size: 16px;
                font-weight: 600;
                color: #F2AA4C;
                border: 1px solid #2B2B2B;
                border-radius: 12px;
                margin-top: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 16px;
                top: 8px;
                padding: 0 4px;
            }
            QPushButton#ListenerButton {
                font-size: 16px;
                font-weight: 600;
                padding: 10px 24px;
                border-radius: 20px;
                background-color: #F2AA4C;
                color: #101820;
            }
            QPushButton#ListenerButton:checked {
                background-color: #FF6F61;
                color: #121212;
            }
            QPushButton {
                font-size: 15px;
                padding: 10px 16px;
                border-radius: 12px;
                background-color: #1F3B4D;
                color: #F8F8F8;
            }
            QPushButton:hover {
                background-color: #29526A;
            }
        """
        )

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

    def set_matlab_window(self, window_title: str | None) -> None:
        text = window_title if window_title else "No MATLAB editor detected"
        self._matlab_label.setText(text)
        status = f"MATLAB window: {text}" if window_title else "Waiting for MATLAB editor..."
        self.statusBar().showMessage(status)

    def set_listener_state(self, active: bool) -> None:
        self._listener_button.setChecked(active)
        self._listener_button.setText("Stop Listening" if active else "Start Listening")
        self._listener_status.setText("Listening" if active else "Idle")

    def log_message(self, message: str) -> None:
        cursor = self._log.textCursor()
        cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
        cursor.insertText(f"{message}\n")
        self._log.setTextCursor(cursor)

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_listener_clicked(self, checked: bool) -> None:
        self.listener_toggled.emit(checked)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # pragma: no cover - Qt lifecycle
        self.listener_toggled.emit(False)
        super().closeEvent(event)
