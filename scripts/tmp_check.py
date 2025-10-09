from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PyQt6 import QtWidgets

from src.ui.main_window import MatlabHelperMainWindow

print("Creating QApplication...")
app = QtWidgets.QApplication(sys.argv)
print("Creating main window...")
window = MatlabHelperMainWindow()
print("Showing window...")
window.show()
print("isVisible before loop:", window.isVisible())
result = app.exec()
print("app.exec() returned", result)
