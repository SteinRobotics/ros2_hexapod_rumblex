"""Write shared CAD parts to both robot export directories."""

from pathlib import Path

from build123d import export_step, export_stl


GENERATED_DIR = Path(__file__).resolve().parents[1] / "generated"


def export_common_part(part, name: str, *, stl: bool = False) -> None:
    for robot in ("nox", "nira"):
        export_dir = GENERATED_DIR / robot
        step_dir = export_dir / "step"
        step_dir.mkdir(parents=True, exist_ok=True)
        export_step(part, str(step_dir / f"{name}.step"))

        if stl:
            stl_dir = export_dir / "stl"
            stl_dir.mkdir(parents=True, exist_ok=True)
            export_stl(part, str(stl_dir / f"{name}.stl"))
