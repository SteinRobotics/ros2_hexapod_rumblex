"""Write shared CAD parts to both robot export directories."""

from pathlib import Path

from build123d import export_step, export_stl
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.TopLoc import TopLoc_Location


GENERATED_DIR = Path(__file__).resolve().parents[1] / "generated"


def export_stl_ignoring_degenerate_faces(part, path: str) -> bool:
    """Export the mesh while omitting only faces too small to triangulate.

    Some imported STEP parts contain zero-area sliver faces. Open Cascade's
    STL writer skips them anyway, but prints a warning for every assembly
    instance. Reject any unmeshed face with meaningful area so a real mesh
    failure cannot silently remove geometry.
    """
    mesh = BRepMesh_IncrementalMesh(part.wrapped, 1e-3, True, 0.1, True)
    mesh.Perform()

    builder = BRep_Builder()
    meshed_faces = TopoDS_Compound()
    builder.MakeCompound(meshed_faces)
    faces = TopExp_Explorer(part.wrapped, TopAbs_FACE)
    while faces.More():
        face = TopoDS.Face_s(faces.Current())
        if BRep_Tool.Triangulation_s(face, TopLoc_Location()) is None:
            properties = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, properties)
            if abs(properties.Mass()) > 1e-8:
                raise ValueError("STL export would omit a non-degenerate face")
        else:
            builder.Add(meshed_faces, face)
        faces.Next()

    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    return writer.Write(meshed_faces, path)


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
