"""Preserve CAD DXF curves; use tolerance-bounded polygons only for nesting."""

from dataclasses import dataclass
from io import BytesIO, StringIO

import ezdxf
from ezdxf.path import make_path
from shapely.geometry import LineString, Polygon
from shapely.ops import polygonize_full


@dataclass
class Profile:
    name: str
    quantity: int
    thickness: float
    polygon: Polygon
    entities: list
    nominal_area: float
    contour_sizes: list


def contour_polygon(entities, tolerance):
    lines = []
    for entity in entities:
        kind = entity.dxftype()
        if kind in ("CIRCLE", "ARC"):
            vertices = list(entity.flattening(tolerance))
        elif kind == "SPLINE":
            vertices = list(entity.flattening(tolerance))
        else:
            vertices = list(make_path(entity).flattening(tolerance))
        if len(vertices) < 2 or any(abs(p.z) > 1e-6 for p in vertices):
            raise ValueError("Empty or non-planar cutting entity")
        points = [(round(p.x, 7), round(p.y, 7)) for p in vertices]
        lines.append(LineString(points))
    polygons, cuts, dangles, invalid = polygonize_full(lines)
    if not cuts.is_empty or not dangles.is_empty or not invalid.is_empty or len(polygons.geoms) != 1:
        raise ValueError("CAD wire does not form exactly one closed, nonintersecting contour")
    return polygons.geoms[0]


def make_profile(component, tolerance=0.01, kerf=0):
    from build123d import ExportDXF, Kind, offset
    if component.quantity <= 0 or type(component.quantity) is not int:
        raise ValueError(f"{component.name}: quantity must be a positive integer")
    if component.thickness not in (1.0, 1.5, 3.0):
        raise ValueError(f"{component.name}: unsupported thickness {component.thickness}")
    faces = component.surface.faces()
    if len(faces) != 1:
        raise ValueError(f"{component.name}: expected one connected physical component, got {len(faces)} faces")
    face = faces[0]
    nominal_area = face.area
    if kerf:
        hole_count = len(face.inner_wires())
        try:
            compensated = offset(face, amount=kerf / 2, kind=Kind.ARC)
        except Exception as error:
            raise ValueError(f"CAD kerf offset failed: {error}") from error
        faces = compensated.faces()
        if len(faces) != 1 or len(faces[0].inner_wires()) != hole_count:
            raise ValueError("Kerf compensation removes a hole or changes topology")
        face = faces[0]
    rings, entities, contour_sizes = [], [], []
    for wire in [face.outer_wire(), *face.inner_wires()]:
        drawing = ExportDXF()
        drawing.add_shape(wire)
        stream = BytesIO()
        drawing.write(stream)
        doc = ezdxf.read(StringIO(stream.getvalue().decode("utf-8")))
        edges = list(doc.modelspace())
        rings.append(contour_polygon(edges, tolerance))
        entities.extend(edges)
        contour_sizes.append(len(edges))
    polygon = Polygon(rings[0].exterior.coords, [r.exterior.coords for r in rings[1:]])
    if not polygon.is_valid or polygon.is_empty:
        raise ValueError(f"{component.name}: invalid or intersecting cutting contours")
    if abs(polygon.area - face.area) > max(0.01, polygon.length * tolerance * 2):
        raise ValueError(f"{component.name}: DXF area does not match CAD face")
    return Profile(component.name, component.quantity, component.thickness,
                   polygon, entities, nominal_area, contour_sizes)
