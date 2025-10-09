"""Tests for the MatlabHelper entry point."""

from __future__ import annotations

from typing import Any, Dict, List

import src.main as main_module


def test_main_exits_cleanly(monkeypatch) -> None:
    created: Dict[str, Any] = {}
    app_ref: Dict[str, Any] = {}

    class DummyApp:
        def __init__(self, args: List[str]) -> None:  # type: ignore[override]
            created["args"] = list(args)
            app_ref["instance"] = self

        def exec(self) -> int:  # type: ignore[override]
            created["exec_called"] = True
            return 0

        @staticmethod
        def instance() -> "DummyApp | None":  # type: ignore[override]
            return app_ref.get("instance")

        def processEvents(self) -> None:  # type: ignore[override]
            created.setdefault("events_processed", 0)
            created["events_processed"] += 1

    class DummyWindow:
        def show(self) -> None:  # type: ignore[override]
            created["window_shown"] = True

    def fake_controller(window) -> None:
        created["controller_window"] = window

    monkeypatch.setattr(main_module.QtWidgets, "QApplication", DummyApp)
    monkeypatch.setattr(main_module, "MatlabHelperMainWindow", lambda: DummyWindow())
    monkeypatch.setattr(main_module, "MainController", fake_controller)

    exit_code = main_module.main()

    assert exit_code == 0
    assert created["exec_called"] is True
    assert created["window_shown"] is True
    assert "controller_window" in created
