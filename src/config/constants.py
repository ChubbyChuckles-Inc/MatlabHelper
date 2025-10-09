"""Global constants and configuration values for MatlabHelper."""

from __future__ import annotations

from pathlib import Path

APP_NAME: str = "MatlabHelper"
SUPPORTED_EXTENSION: str = ".m"
DEFAULT_WORKDIR: Path = Path.home()
MATLAB_WINDOW_KEYWORDS: tuple[str, ...] = ("MATLAB", "Editor")
INJECTION_PRE_FOCUS_DELAY_SECONDS: float = 0.2
