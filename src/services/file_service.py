"""Utilities for validating and loading MATLAB script files."""

from __future__ import annotations

from pathlib import Path

from src.config.constants import SUPPORTED_EXTENSION


class MatlabFileError(Exception):
    """Raised when a MATLAB source file cannot be processed."""


def ensure_matlab_file(path: Path) -> Path:
    """Ensure *path* points to a readable MATLAB ``.m`` file.

    Parameters
    ----------
    path:
        Candidate path selected by the user.

    Returns
    -------
    Path
        A normalised, absolute path to the MATLAB file.

    Raises
    ------
    MatlabFileError
        If the file does not exist, is not readable, or bears the wrong extension.
    """

    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise MatlabFileError(f"File not found: {resolved}")
    if resolved.suffix.lower() != SUPPORTED_EXTENSION:
        raise MatlabFileError(
            f"Unsupported file extension '{resolved.suffix}'. Expected '{SUPPORTED_EXTENSION}'."
        )
    if resolved.stat().st_size == 0:
        raise MatlabFileError("MATLAB script is empty; please select a non-empty file.")
    return resolved


def read_matlab_source(path: Path, *, encoding: str = "utf-8") -> str:
    """Return the text contents of a validated MATLAB script."""

    validated = ensure_matlab_file(path)
    try:
        return validated.read_text(encoding=encoding)
    except OSError as exc:
        raise MatlabFileError(f"Failed to read MATLAB file: {validated}") from exc
