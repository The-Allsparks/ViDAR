"""Smoke tests for the CLI entry point."""
from vidar.main import run
from vidar.output.telemetry import TelemetryPublisher


def test_main_entry_symbols_importable():
    assert callable(run)
    assert callable(TelemetryPublisher)
