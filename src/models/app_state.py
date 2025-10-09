"""Persistent application state model for MatlabHelper."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

from src.models.typing_settings import TypingSettings


_LOG_LIMIT = 200


@dataclass
class AppState:
    """Serializable snapshot of UI-facing application state."""

    typing_settings: TypingSettings = field(default_factory=TypingSettings.from_defaults)
    selected_file: Optional[str] = None
    selected_file_mtime: Optional[float] = None
    typed_index: int = 0
    listener_active: bool = False
    target_window_title: Optional[str] = None
    target_window_handle: Optional[int] = None
    window_geometry: Optional[str] = None
    window_maximized: bool = False
    active_tab: int = 0
    log_messages: list[str] = field(default_factory=list)
    status_message: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "typing_settings": self.typing_settings.to_dict(),
            "selected_file": self.selected_file,
            "selected_file_mtime": self.selected_file_mtime,
            "typed_index": self.typed_index,
            "listener_active": self.listener_active,
            "target_window_title": self.target_window_title,
            "target_window_handle": self.target_window_handle,
            "window_geometry": self.window_geometry,
            "window_maximized": self.window_maximized,
            "active_tab": self.active_tab,
            "log_messages": self._trimmed_logs(),
            "status_message": self.status_message,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AppState":
        instance = cls()
        typing_settings_payload = data.get("typing_settings")
        if isinstance(typing_settings_payload, Mapping):
            instance.typing_settings = TypingSettings.from_dict(typing_settings_payload)
        instance.selected_file = cls._coerce_optional_str(data.get("selected_file"))
        instance.selected_file_mtime = cls._coerce_optional_float(data.get("selected_file_mtime"))
        instance.typed_index = cls._coerce_int(data.get("typed_index"), default=0)
        instance.listener_active = bool(data.get("listener_active", False))
        instance.target_window_title = cls._coerce_optional_str(data.get("target_window_title"))
        instance.target_window_handle = cls._coerce_optional_int(data.get("target_window_handle"))
        instance.window_geometry = cls._coerce_optional_str(data.get("window_geometry"))
        instance.window_maximized = bool(data.get("window_maximized", False))
        instance.active_tab = cls._coerce_int(data.get("active_tab"), default=0)
        log_messages = data.get("log_messages", [])
        if isinstance(log_messages, list):
            instance.log_messages = [str(message) for message in log_messages][-_LOG_LIMIT:]
        instance.status_message = cls._coerce_optional_str(data.get("status_message"))
        return instance

    def update_logs(self, message: str) -> None:
        self.log_messages.append(message)
        if len(self.log_messages) > _LOG_LIMIT:
            self.log_messages = self.log_messages[-_LOG_LIMIT:]

    def _trimmed_logs(self) -> list[str]:
        return self.log_messages[-_LOG_LIMIT:]

    @staticmethod
    def _coerce_optional_str(value: Any) -> Optional[str]:
        if value is None:
            return None
        return str(value)

    @staticmethod
    def _coerce_optional_float(value: Any) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _coerce_int(value: Any, *, default: int = 0) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _coerce_optional_int(value: Any) -> Optional[int]:
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None
