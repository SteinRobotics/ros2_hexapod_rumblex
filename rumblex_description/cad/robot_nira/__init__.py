"""Nira CAD models and their shared export directory."""

from pathlib import Path

EXPORT_DIR = Path(__file__).resolve().parents[1] / "generated" / "nira"
