"""Global constants and configuration values for MatlabHelper."""

from __future__ import annotations

from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent.parent


APP_NAME: str = "MatlabHelper"
SUPPORTED_EXTENSION: str = ".m"
DEFAULT_WORKDIR: Path = Path.home()
MATLAB_WINDOW_KEYWORDS: tuple[str, ...] = ("MATLAB", "Editor")
ICON_DIR: Path = _project_root() / "icons"
MAX_PREVIEW_CHARS: int = 24
INJECTION_PRE_FOCUS_DELAY_SECONDS: float = 0.12
TYPE_CHAR_DELAY_SECONDS: float = 0.0  # retained for compatibility but unused by injector
MIN_CHAR_DELAY_SECONDS: float = 0.0
MAX_CHAR_DELAY_SECONDS: float = 0.06
TYPING_ERROR_PROBABILITY: float = 0.18
TYPING_ERROR_MIN_BURST: int = 1
TYPING_ERROR_MAX_BURST: int = 3
TEMPO_REFERENCE_SECONDS: float = 0.35
TEMPO_MIN_FACTOR: float = 0.05
TEMPO_MAX_FACTOR: float = 1.35
