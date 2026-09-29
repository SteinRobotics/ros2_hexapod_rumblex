#!/usr/bin/env python3
"""Export Nira assemblies and copy their STL files to the description meshes."""

from pathlib import Path
from shutil import copy2

from robot_nira import EXPORT_DIR
from robot_nira import assembly_complete, assembly_coxa, assembly_femur, assembly_tibia, assembly_torso_with_servos


def main() -> None:
    for assembly in (assembly_complete, assembly_coxa, assembly_femur, assembly_tibia, assembly_torso_with_servos):
        assembly.main()

    mesh_dir = Path(__file__).resolve().parent.parent / "meshes" / "nira"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for stl_file in sorted((EXPORT_DIR / "stl").glob("*.stl")):
        copy2(stl_file, mesh_dir / stl_file.name)


if __name__ == "__main__":
    main()
