"""Pytest configuration for MatlabHelper."""

from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.fixture(autouse=True)
def _ensure_qt_offscreen() -> None:
    """Force Qt to use an offscreen platform during tests."""
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
