"""File-backed persistence for :class:`AppState`."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Optional

from src.config import constants
from src.models.app_state import AppState

_LOGGER = logging.getLogger(__name__)


class AppStateStore:
    """Serialize and deserialize application state using JSON."""

    def __init__(self, path: Optional[Path] = None) -> None:
        self._path = Path(path) if path else constants.APP_STATE_PATH

    def load(self) -> AppState:
        try:
            with self._path.open("r", encoding="utf-8") as handle:
                payload = json.load(handle)
        except FileNotFoundError:
            return AppState()
        except Exception as exc:  # pragma: no cover - unexpected format
            _LOGGER.warning("Failed to load app state: %s", exc)
            return AppState()
        if not isinstance(payload, dict):
            return AppState()
        return AppState.from_dict(payload)

    def save(self, state: AppState) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        payload = state.to_dict()
        tmp_path = self._path.with_suffix(".tmp")
        try:
            with tmp_path.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
            tmp_path.replace(self._path)
        except Exception as exc:  # pragma: no cover - filesystem issues
            _LOGGER.error("Failed to persist app state: %s", exc)
            if tmp_path.exists():
                tmp_path.unlink(missing_ok=True)
