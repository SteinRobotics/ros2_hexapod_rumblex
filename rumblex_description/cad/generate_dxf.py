#!/usr/bin/env python3
"""Generate complete, thickness-separated laser cutting jobs from robot CAD."""

import argparse
from pathlib import Path
import sys
from laser_cutting.robots import ROBOT_BUILDERS


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Example for Nira:\n  .venv/bin/python generate_dxf.py --robot nira --output generated/nira/",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--robot", required=True, choices=tuple(ROBOT_BUILDERS))
    parser.add_argument("--output", type=Path, default=Path("output"), help="New output root (default: output)")
    parser.add_argument("--sheet-width", type=float, default=300, help="Material sheet width, mm (default: 300)")
    parser.add_argument("--sheet-height", type=float, default=300, help="Material sheet height, mm (default: 300)")
    parser.add_argument("--clearance", type=float, default=1, help="Minimum gap between cuts, mm (default: 1)")
    parser.add_argument("--margin", type=float, default=3, help="Sheet edge margin, mm (default: 3)")
    parser.add_argument("--kerf", type=float, default=0, help="Full beam width, mm; 0 preserves CAD curves")
    parser.add_argument("--resolution", type=float, default=1, help="Nesting grid, mm (default: 1)")
    parser.add_argument("--rotation-step", type=float, default=30, help="Rotation search increment, degrees")
    parser.add_argument("--tolerance", type=float, default=0.01, help="Curve sampling tolerance, mm")
    parser.add_argument("--trials", type=int, default=3, help="Number of deterministic nesting orders")
    args = parser.parse_args(argv)
    try:
        from laser_cutting.geometry import make_profile
        from laser_cutting.nesting import Settings, nest
        from laser_cutting.output import write_job
        from laser_cutting.robots import robot_components
        settings = Settings(width=args.sheet_width, height=args.sheet_height,
                            clearance=args.clearance, margin=args.margin, kerf=args.kerf,
                            resolution=args.resolution, rotation_step=args.rotation_step,
                            tolerance=args.tolerance, trials=args.trials)
        settings.validate()
        if (args.output / args.robot).exists():
            raise ValueError(f"{args.output / args.robot} already exists; choose a new --output directory")
        print(f"Building {args.robot} laser-cut BOM from CAD...", flush=True)
        components = robot_components(args.robot)
        names = [c.name for c in components]
        if not names or len(names) != len(set(names)):
            raise ValueError("Empty BOM or duplicate component type names")
        profiles = []
        for component in components:
            try:
                profiles.append(make_profile(component, args.tolerance, args.kerf))
            except Exception as error:
                raise ValueError(f"{component.name}: {error}") from error
        layouts = {}
        for thickness in sorted({p.thickness for p in profiles}):
            group = [p for p in profiles if p.thickness == thickness]
            print(f"Nesting {sum(p.quantity for p in group)} parts in {thickness:g} mm material...", flush=True)
            layouts[thickness] = nest(group, settings)
        destination, report = write_job(args.robot, profiles, layouts, settings, args.output)
        print(f"Validated {report['placed_components']}/{report['required_components']} components.")
        for thickness, count in report["sheets_per_thickness"].items():
            print(f"  {thickness}: {count} sheet(s)")
        print(f"DXFs and reports: {destination}")
        return 0
    except (ValueError, ImportError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
