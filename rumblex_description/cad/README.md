# RumbleX CAD (build123d)

Run scripts from this directory with Python 3.10+ and `build123d` installed.
`ocp_vscode` is optional for viewing. Vendor STEP files live in `imported/`;
Nox scripts export to `generated/nox/`; shared component scripts export to
`generated/` when run directly. Run `robot_nox.py` to export and display the
complete Nox assembly from `robot_nox/assembly_complete.py`.
Scripts in subfolders also support direct execution, for example
`python robot_nox/assembly_complete.py` or `python common/toe.py`.

`robot_nox/chassis_side.py` exports the rectangular front/back plate (`chassis_side_end`)
and inverted-U left/right plate (`chassis_side`) as STEP and flat DXF files.
The body assembly places two of each between layers 1 and 3, passing through
layer 2's long slots. Tabs match the nominal 8 x 1.5 mm slots; no manufacturing
clearance or kerf compensation is applied.

Four rectangular `chassis_side_diagonal` plates span the 45 mm gap between
layers 2 and 3, with three tabs at each end. Their matching diagonal slot rows
follow the inner octagonal opening with the same clearance as the horizontal
rows. The plate body is 40 x 45 x 1.5 mm; including tabs, its height is 48 mm.

```bash
source .venv/bin/activate
python -m robot_nox.assembly_leg
python robot_nox.py
python -m unittest discover -s tests -p test_cad_geometry.py -v
```

## Dimensions and joints

All lengths are in millimetres and angles in degrees. Servo dimensions and
mounting-hole positions and shared cutout profiles are defined in `servo_simplified.py`. Foot and body
cutout hole patterns use those same dimensions; drawing polygons retain their
original coordinates with `align=None`.

The servo rotation joint is at the front horn's outer mounting face. Bracket
joints are at the matching inner leg face, centred on the shaft hole. The
37 mm bracket opening matches the distance between the servo's outer horn
faces. Assemblies connect these joints directly, without translations or
rotations after connection. Inclined brackets also expose a `plate_mount`
joint at the centre of their outer top face.

The femur is rolled 180 degrees about its longitudinal (local Z) axis through
the midpoint between the horn faces. Its rear horn mates with the coxa; the
outgoing tibia interface follows the rolled bracket.
The tibia is likewise rolled 180 degrees about its lengthwise axis (local X),
through the horn midpoint at shaft height, so its rear horn mates with the femur.

`assembly_leg.attach_leg()` is shared by the standalone leg and complete robot.
Angles specify rotation about the mating axis; negative angles are supported.
The display pose uses 90 degrees at the femur and -90 degrees at the tibia to
place the feet below the body. These are pose settings, not dimensional fit
corrections; the old corrected zero-angle pose is not preserved.

## Remaining vendor fits

The bottom bracket's 5.65 mm insertion and the head side bracket's registration
offset are retained empirical fits to vendor STEP geometry. They still need
physical verification. The separate `assembly_tibia_HX35H.py` vendor-servo
example also retains its original offsets.

The headless tests check joint coincidence and shaft alignment, foot mounting
hole alignment, bracket width, drawing coordinates, solid validity and the
complete robot's default foot placement. They do not certify collision-free
motion or manufacturing tolerances.

## Nira forward lidar deck

Nira's T-mini sits directly on layer 2 at **(60, 0, 51.5) mm**, fully within
the front third of the nominal 220 mm body (+X is forward). Its scan plane
is at Z=77.8 mm. Layer 2 has an integral bridge across the former opening,
two 22 x 3 mm adapter slots beside the housing, and a rear cable opening.
The slots are generic attachment provisions: the simplified sensor model
has no vendor mounting-hole or connector geometry, so the final adapter
and cable routing still require physical verification.

The upper body forms a swept rear canopy with paired ventilation gills,
a solid central spine, and a burgundy layer-3 frame below the cream roof.
The front and front-diagonal walls end 8 mm above the deck. Side shoulders
sweep back to full height, retaining their three rear roof tabs; the rear
walls, diagonal braces and four rear upper spacers support the canopy.
Layer 4 now has through-holes aligned with those spacers.

The forward **270° sector (-135° to +135° from +X)** is clear in the default
complete-robot pose. The geometry test intersects a continuous 4 mm high
scan band with the robot, from 20 to approximately 600 mm radius. It also
checks full sensor-footprint support, single-solid validity, body collisions,
and retained tab engagement. This does not verify clearance throughout
head/leg motion, structural strength, optical tolerances or manufacturing fit.

Export the revised parts and assembly with:

```bash
.venv/bin/python -m robot_nira.body_layer_2
.venv/bin/python -m robot_nira.body_layer_3
.venv/bin/python -m robot_nira.body_layer_4
.venv/bin/python -m robot_nira.chassis_side
.venv/bin/python -m robot_nira.assembly_body
.venv/bin/python robot_nira.py
.venv/bin/python -m unittest discover -s tests -p test_nira_lidar.py -v
```

Outputs are under `generated/nira/`. Side exports distinguish `chassis_front`,
`chassis_back`, `chassis_side`, `chassis_diagonal_front`, and
`chassis_diagonal_back`; the front and rear parts are no longer interchangeable.
Each plate has a STEP model and a flat DXF drawing. The complete STEP includes
the lidar. `lidar_body_preview.png` shows the redesigned body.
