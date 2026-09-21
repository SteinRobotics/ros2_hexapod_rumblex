# RumbleX CAD (build123d)

Run scripts from this directory with Python 3.10+ and `build123d` installed.
`ocp_vscode` is optional for viewing. Vendor STEP files live in `imported/`;
scripts export to `generated/` when run directly.

```bash
source .venv/bin/activate
python assembly_leg.py
python assembly_complete.py
python -m unittest test_cad_geometry -v
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
