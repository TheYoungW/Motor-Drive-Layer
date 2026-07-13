from __future__ import annotations

import motorbridge.platform_hints as hints


def test_linux_classic_can_hint_is_self_contained() -> None:
    message = hints.linux_socketcan_hint("motorbridge-cli", "can0", "socketcan")

    assert "scripts/" not in message
    assert "bitrate 1000000" in message
    assert "dbitrate" not in message


def test_linux_canfd_hint_includes_data_bitrate() -> None:
    message = hints.linux_socketcan_hint("motorbridge-cli", "can0", "socketcanfd")

    assert "dbitrate 5000000 fd on" in message


def test_non_linux_socketcan_is_reported_as_unsupported(monkeypatch) -> None:
    monkeypatch.setattr(hints, "is_linux", lambda: False)

    message = hints.preflight_can_runtime("motorbridge-cli", "socketcan", "can0")

    assert message is not None
    assert "only available on Linux" in message
