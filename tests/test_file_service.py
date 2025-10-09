"""Tests for MATLAB file validation and reading utilities."""

from __future__ import annotations

from pathlib import Path

import pytest

from src.services.file_service import MatlabFileError, ensure_matlab_file, read_matlab_source


def test_ensure_matlab_file_accepts_valid_script(tmp_path: Path) -> None:
    script = tmp_path / "demo.m"
    script.write_text("disp('hello world');\n")

    validated = ensure_matlab_file(script)

    assert validated == script.resolve()


def test_ensure_matlab_file_rejects_wrong_extension(tmp_path: Path) -> None:
    script = tmp_path / "demo.txt"
    script.write_text("% not actually matlab")

    with pytest.raises(MatlabFileError):
        ensure_matlab_file(script)


def test_read_matlab_source_returns_content(tmp_path: Path) -> None:
    script = tmp_path / "demo.m"
    content = "% comment\nvalue = 42;\n"
    script.write_text(content)

    assert read_matlab_source(script) == content


def test_read_matlab_source_raises_for_missing_file(tmp_path: Path) -> None:
    script = tmp_path / "missing.m"
    with pytest.raises(MatlabFileError):
        read_matlab_source(script)
