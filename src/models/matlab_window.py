"""Data models representing MATLAB window metadata."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MatlabWindowInfo:
    """Metadata about a detected MATLAB editor window."""

    handle: int
    title: str

    def is_valid(self) -> bool:
        """Return ``True`` when the window metadata looks complete."""
        return self.handle != 0 and bool(self.title.strip())
