# Hexapod Leg CAD (build123d)

Parametric CAD scripts for a hexapod leg plate, spacer, and servo-mounted assemblies using build123d.

## Project Layout

- `leg_top.py`: 2D plate sketch + extrusion, exports STEP and DXF
- `spacer.py`: cylindrical spacer model with center hole, exports STEP and STL
- `assembly_leg_with_spacers.py`: leg plate + spacer assembly
- `assembly_leg_with_servo_HX35H.py`: HX-35H servo + leg assembly
- `assembly_leg_with_servo_ST3215.py`: ST3215 servo + leg assembly
- `assembly_leg_with_servo_and_bracket.py`: ST3215 servo + leg + imported bracket assembly
- `cad_config.py`: shared dimensions and placement offsets
- `ocp_utils.py`: optional viewer helper for OCP CAD Viewer
- `imported/`: vendor STEP assets used by assemblies
- `generated/`: exported output files

## Requirements

- Python 3.10+
- build123d
- ocp_vscode (optional, for in-editor viewing)

Install:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install build123d ocp_vscode
```

## Usage

Run any script directly to generate outputs into `generated/`.

```bash
python leg_top.py
python spacer.py
python assembly_leg_with_spacers.py
python assembly_leg_with_servo_HX35H.py
python assembly_leg_with_servo_ST3215.py
python assembly_leg_with_servo_and_bracket.py
```

## Shared Parameters

Common dimensions and placement offsets are centralized in `cad_config.py`.

- Spacer dimensions:
  - `FOOT_SPACER_OUTER_DIAMETER`
  - `SPACER_STUD_HOLE_DIAMETER`
  - `SPACER_OVERALL_LENGTH`
- Assembly placement offsets:
  - `ST3215_LEG_OFFSET_X`
  - `ST3215_LEG_OFFSET_Z`
  - `HX35H_LEG_OFFSET_X`
  - `HX35H_LEG_OFFSET_Y`
  - `HX35H_LEG_OFFSET_Z`

Adjusting these values updates all consumers that import them.

## Notes

- Imported STEP assets are expected at fixed paths under `imported/`.
- `ocp_utils.show(...)` is safe to call even when the viewer backend is not running; it no-ops in that case.
