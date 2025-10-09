"""Build script for generating MSI installers with cx_Freeze."""

from __future__ import annotations

from pathlib import Path

from src.config.constants import APP_NAME

try:  # pragma: no cover - only executed during packaging
    from cx_Freeze import Executable, setup  # type: ignore[import]
except ImportError as exc:  # pragma: no cover - packaging dependency resolution
    raise SystemExit("Install cx-Freeze to build the MatlabHelper installer.") from exc


ICON_PATH = Path("docs/source/_static/icon.ico")

build_exe_options = {
    "packages": ["keyboard", "PyQt6", "win32gui", "win32con", "win32clipboard"],
    "includes": ["src"],
    "include_files": [],
}

bdist_msi_options = {
    "upgrade_code": "{7D1E385F-540D-4EF5-B4F2-39C3F8C9D91A}",
    "install_icon": str(ICON_PATH) if ICON_PATH.exists() else None,
}

setup(
    name=APP_NAME,
    version="0.1.0",
    description="Matlab teaching assistant that streams scripts into the MATLAB editor.",
    options={"build_exe": build_exe_options, "bdist_msi": bdist_msi_options},
    executables=[
        Executable(
            script="src/main.py",
            base="Win32GUI",
            target_name=f"{APP_NAME}.exe",
        )
    ],
)
