"""Entry point for the MatlabHelper PyQt application."""

from __future__ import annotations

import sys

from PyQt6 import QtWidgets

from src.app.controller import MainController
from src.ui.main_window import MatlabHelperMainWindow


def main() -> int:
    """Launch the PyQt6 application."""

    app = QtWidgets.QApplication(sys.argv)
    window = MatlabHelperMainWindow()
    MainController(window)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
