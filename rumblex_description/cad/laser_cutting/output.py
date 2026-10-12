"""Validated, transactional DXF output and machine-readable cutting report."""

from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import shutil
import tempfile

import ezdxf
from ezdxf.math import Matrix44
from shapely.geometry import Polygon
from .geometry import contour_polygon


def write_job(robot, profiles, layouts, settings, output):
    destination = Path(output).resolve() / robot
    if destination.exists():
        raise ValueError(f"Output already exists: {destination}. Choose a new --output directory.")
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{robot}-", dir=destination.parent))
    report = {
        "robot": robot, "units": "mm", "scope": "laser-cut sheet components only",
        "settings": asdict(settings), "required_components": sum(p.quantity for p in profiles),
        "components": [{"name": p.name, "quantity": p.quantity, "thickness_mm": p.thickness,
                        "nominal_area_mm2": p.nominal_area} for p in profiles],
        "nesting": "deterministic multi-order conservative polygon raster heuristic; no optimality guarantee",
        "geometry": ("original CAD curves" if not settings.kerf else
                     "CAD curves offset by half the kerf, with round corner joins"),
        "sheets_per_thickness": {}, "sheets": [],
    }
    try:
        for thickness, sheets in sorted(layouts.items()):
            folder = f"{thickness:g}mm"
            report["sheets_per_thickness"][folder] = len(sheets)
            (staging / folder).mkdir()
            for index, sheet in enumerate(sheets, 1):
                relative = f"{folder}/sheet_{index:02d}.dxf"
                doc = ezdxf.new("R2010")
                doc.units = ezdxf.units.MM
                doc.header["$MEASUREMENT"] = 1
                model = doc.modelspace()
                placements = []
                for part in sheet:
                    layer = f"{part.profile.name}__{part.instance:03d}"
                    doc.layers.new(layer)
                    matrix = Matrix44.chain(Matrix44.z_rotate(math.radians(part.angle)),
                                            Matrix44.translate(part.x, part.y, 0))
                    for source in part.profile.entities:
                        entity = source.copy()
                        entity.transform(matrix)
                        entity.dxf.layer = layer
                        model.add_entity(entity)
                    placements.append({"type": part.profile.name, "instance": part.instance,
                                       "layer": layer, "rotation_deg": part.angle,
                                       "translation_mm": [part.x, part.y]})
                path = staging / relative
                doc.saveas(path)
                loaded = ezdxf.readfile(path)
                if loaded.units != ezdxf.units.MM or loaded.audit().has_errors:
                    raise ValueError(f"DXF audit failed: {relative}")
                counts = Counter(e.dxf.layer for e in loaded.modelspace())
                for part, placement in zip(sheet, placements):
                    if counts[placement["layer"]] != len(part.profile.entities):
                        raise ValueError(f"DXF entity count changed for {placement['layer']}")
                    edges = [e for e in loaded.modelspace() if e.dxf.layer == placement["layer"]]
                    rings, start = [], 0
                    for size in part.profile.contour_sizes:
                        rings.append(contour_polygon(edges[start:start+size], settings.tolerance))
                        start += size
                    polygon = Polygon(rings[0].exterior.coords, [r.exterior.coords for r in rings[1:]])
                    if (not polygon.is_valid or len(polygon.interiors) != len(part.polygon.interiors)
                            or polygon.symmetric_difference(part.polygon).area >
                            max(0.001, settings.tolerance * polygon.length * 2)):
                        raise ValueError(f"DXF read-back geometry mismatch: {placement['layer']}")
                report["sheets"].append({"file": relative, "thickness_mm": thickness,
                    "placed_components": len(sheet),
                    "utilization_percent": 100 * sum(p.profile.nominal_area for p in sheet)
                        / (settings.width * settings.height), "placements": placements})
        report["placed_components"] = sum(s["placed_components"] for s in report["sheets"])
        if report["placed_components"] != report["required_components"]:
            raise ValueError("Exported component count differs from BOM")
        (staging / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        lines = [f"Robot: {robot}", f"Laser-cut components: {report['placed_components']}",
                 "Dimensions: millimetres, 1:1", "", "Bill of materials:"]
        lines += [f"  {p.name}: {p.quantity} x {p.thickness:g} mm" for p in profiles]
        lines += ["", "Sheets:"]
        lines += [f"  {s['file']}: {s['placed_components']} parts, "
                  f"{s['utilization_percent']:.2f}% material utilization" for s in report["sheets"]]
        lines += ["", report["nesting"], f"Geometry: {report['geometry']}"]
        (staging / "summary.txt").write_text("\n".join(lines) + "\n")
        staging.rename(destination)
    except BaseException:
        shutil.rmtree(staging)
        raise
    return destination, report
