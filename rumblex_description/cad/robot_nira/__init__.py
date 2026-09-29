"""Nira CAD models and their shared export directory."""

from pathlib import Path

EXPORT_DIR = Path(__file__).resolve().parents[1] / "generated" / "nira"


def dxf_directory(thickness: float, *, export_dir: Path = EXPORT_DIR) -> Path:
    """Return the DXF folder for the required material thickness in millimetres."""
    folders = {1.0: "dxf_1mm", 1.5: "dxf_1p5mm", 3.0: "dxf_3mm"}
    directory = export_dir / "dxf" / folders[thickness]
    directory.mkdir(parents=True, exist_ok=True)
    return directory
