# RumbleX CAD (build123d)

Run scripts from this directory with Python 3.10+ and `build123d` installed.
`ocp_vscode` is optional for viewing. Vendor STEP files live in `imported/`;
Nox scripts export to `generated/nox/`; scripts in `common/` export their parts
to both `generated/nox/` and `generated/nira/` when run directly. Run
`robot_nox.py` to export and display the complete Nox assembly from
`robot_nox/assembly_complete.py`.
STEP files go in each export directory's `step/` folder, DXF files in `dxf/`,
and STL files in `stl/`. Nira uses `generated/nira/` with the same layout.
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
python -m unittest discover -s tests -v
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

Nox retains the bottom bracket's 5.65 mm insertion and the head side bracket's
empirical registration offset. The separate `assembly_tibia_HX35H.py`
vendor-servo examples also retain their original offsets.

Nira's `vendor_brackets.py` measures mounting-hole centres and planar mounting
faces from the vendor STEP files. Both brackets expose `servo_mount` and
`plate_mount` joints. The side bracket aligns its vertical hole pair with the
servo's rear flange; the bottom bracket aligns its horizontal pair with the
servo's lower front row. Its outer plate joint seats the inclined bracket.
The camera adapter bores follow the side bracket's measured hole pattern.
Spline hole outlines in the bottom STEP are measured by enclosed area and
centroid. Feature selection checks the hole count and servo mounting pitch,
and rejects missing or incompatible patterns. Physical fit still needs verification.

## Nira shared layout

- `body_layout.py` defines body layer elevations, spacer lengths, the cover's
  seating offset and the front deck boundary. Body, servo and standalone
  chassis assemblies use these same elevations.
- Chassis edges follow slot-row centres and widths. Canopy corners follow
  the body width, height and chamfer; the removable cover shares the deck boundary.
- `board_layout.py` lists every PCB placement and hole pattern for both the
  base plate and populated assembly.
- `foot_common.CHEEK_OUTER_SPAN` sizes both the foot armor and toe connection;
  connection notches follow cheek thickness.
- Body toe positions and hole radius are separate. Toe mounting faces sit
  against the underside of layer 0, independent of the bore radius.

The bracket mounting frames replace empirical placement, so the bottom
bracket and outgoing femur interface move slightly. The body toes also move
up 1 mm to contact the base plate. Material thicknesses, clearances, decorative
outlines and display poses remain explicit design parameters.

Run the shared-layout and mounting regression tests with:

```bash
.venv/bin/python -m unittest discover -s tests -p test_nira_layout.py -v
```

The headless tests check joint coincidence and shaft alignment, foot mounting
hole alignment, bracket width, drawing coordinates, solid validity and the
complete robot's default foot placement. They do not certify collision-free
motion or manufacturing tolerances.

## Nira forward lidar deck

Nira's T-mini sits on its interface housing at **(93, 0, 67) mm**
(+X is forward), with its scan plane at Z=93.3 mm. The interface module
rests on layer 2 inside a cream, open-bottom `lidar_interface_housing`.
Its five laser-cut panels have finger joints, a cable opening and six tabs
that engage the deck. Twelve wall tabs enter the lid's edge slots and finish
flush with its upper face. The cover exports STEP/STL and five flat-panel
DXFs under `generated/nira/dxf/lidar_interface_housing/`.
Fits are nominal, without kerf allowance; interface dimensions remain
photo-based estimates pending hardware measurement.

The rear layer-3 frame ends at X=0; the cream layer-4 roof ends at X=80 mm.
The upper spacers are 50 mm long, placing the frame underside at Z=101.5 mm
and roof underside at Z=103 mm. The roof has 2.1 mm vertical clearance above
the scanner envelope. The frame is behind the scanner, so its physical
separation is checked using the solids. The front diagonal braces have two
bottom tabs and a compact solid profile. Rear walls, diagonal braces and four
rear upper spacers support the canopy.

The forward **270° sector (-135° to +135° from +X)** is clear in the default
complete-robot pose. The geometry test intersects a continuous 4 mm high
scan band with the robot, from 20 to approximately 600 mm radius. It also
checks interface seating, housing bores and connector access, canopy-tip
alignment, scanner clearance, single-solid components, body collisions,
and retained tab engagement. This does not verify clearance throughout
head/leg motion, structural strength, optical tolerances or manufacturing fit.

Export the revised parts and assembly with:

```bash
.venv/bin/python -m robot_nira.body_layer_2
.venv/bin/python -m robot_nira.body_layer_3
.venv/bin/python -m robot_nira.body_layer_4
.venv/bin/python -m robot_nira.chassis_back
.venv/bin/python -m robot_nira.chassis_front
.venv/bin/python -m robot_nira.chassis_side
.venv/bin/python -m robot_nira.chassis_diagonal_back
.venv/bin/python -m robot_nira.chassis_diagonal_front
.venv/bin/python -m robot_nira.chassis_slope_cover
.venv/bin/python -m robot_nira.lidar_interface_housing
.venv/bin/python -m robot_nira.assembly_body
.venv/bin/python robot_nira.py
.venv/bin/python -m unittest discover -s tests -p test_nira_lidar.py -v
```

Outputs are under `generated/nira/`. Side exports distinguish `chassis_front`,
`chassis_back`, `chassis_side`, `chassis_diagonal_front`, and
`chassis_diagonal_back`; the front and rear parts are no longer interchangeable.
Each plate has a STEP model and a flat DXF drawing. The complete STEP includes
the lidar and interface housing. `lidar_body_preview.png` shows the body and
an exploded view of the sensor, cover, and interface module.

## Nira vented foot armor

The foot cheeks use a swept toe outline, a broad lower rail and four angled
vents matching the layer-4 canopy. A new cream `foot_outer` plate bridges the
cheeks on their outer (negative drawing-Y) edge. Its paired gills surround a
solid spine, with clipped ends keeping the servo and toe connection accessible.
The existing servo cutouts, mounting holes, spacers and toe interface are retained.

The 1.5 mm outer plate has three 8 mm fingers on each edge, seated in matching
through-slots in the cheeks. The fingers finish flush with the outside faces;
the clear span is 32 mm and overall height is 35 mm. Fits are nominal, without
kerf compensation; physical fit and retention need checking for the chosen
material and laser process.

```bash
.venv/bin/python -m robot_nira.foot_front
.venv/bin/python -m robot_nira.foot_back
.venv/bin/python -m robot_nira.foot_outer
.venv/bin/python -m robot_nira.assembly_foot
.venv/bin/python -m unittest discover -s tests -p test_nira_foot.py -v
```

STEP and flat DXF exports are under `generated/nira/step/` and
`generated/nira/dxf/`; `foot_preview.png` shows
the assembled armor and the three flat profiles. Geometry tests check valid
single-solid plates, six fully engaged tabs, flush ends and outer-plate
clearance from the foot hardware and tibia servo. They do not establish load
capacity or clearance throughout leg motion.

## Nira test scope

Run all Nira tests with:

```bash
.venv/bin/python -m unittest discover -s tests -p 'test_nira_*.py' -v
```

Keep the four suites: foot interfaces and tab engagement (`test_nira_foot`),
camera support and cable access (`test_nira_head`), shared mounting geometry
(`test_nira_layout`), and housing/chassis fit plus optical clearance
(`test_nira_lidar`). They exercise built geometry and mechanical relationships.
The fixed foot mounting coordinates intentionally protect the existing hardware
interface. Decorative opening counts are not a mechanical contract.

The canopy tests require physical separation and a roof above the scanner;
the continuous scan-band test separately protects the optical path. They do
not impose the former 3 mm envelope gap or certify manufacturing clearance.
The head cage collision test excludes the vendor bracket/simplified servo pair,
whose case and flange overlap slightly; separate tests verify its bore alignment.
