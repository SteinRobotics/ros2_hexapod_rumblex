# Movement naming conventions

Use `snake_case` for variables and data fields; private members have a trailing
underscore. Use camelCase for methods, PascalCase for types and enum values,
and `C<Name>Gait` for gait classes. Multiword filenames use underscores, such as
`gait_torso_roll.cpp`, `gait_high_five.hpp`, and `gait_controller.hpp`.
The bundled Feetech library retains its upstream API and naming.

Kinematics methods name the quantity they operate on: `getToePositions`,
`getStandingToePositions`, `getLaydownToePositions`, `setToePosition`,
`getLegAngles`, `getTorsoPose`, and `getHeadOrientation`. Torso poses exclude the
head. Public `CLeg` fields are `angles` and `toe_position`.

ROS parameter components and leg names use lowercase snake_case. Gait fields
are grouped under `gait.<gait_name>`, shared defaults under `gait.generic`, and
servo fields under `servo`. Bare numeric parameters include unit suffixes where
applicable: `_m`, `_deg`, `_rad`, `_m_s`, and `_rad_s`. Typed C++ quantities
already carry their units.

`velocity_to_phase_gain` scales the phase increment per update; it is not a
cycle duration. `transition_phase_span` is an angular span in radians.
The numeric values and update formulas are unchanged.

# Configuration migration

The old parameter keys are replaced, without aliases. Update external parameter
files and command-line overrides using this table. Leg keys and `leg_names`
values change from names such as `RightFront` to `right_front`; C++ `ELegIndex`
enum values retain PascalCase. Within `leg_offsets`, `x_m`, `y_m`, and `yaw_deg`
describe the torso center to coxa mount offset.

| Former parameter/component | New parameter/component |
| --- | --- |
| `CENTER_TO_COXA_X` | `x_m` |
| `CENTER_TO_COXA_Y` | `y_m` |
| `COXA_HEIGHT` | `coxa_height_m` |
| `COXA_LENGTH` | `coxa_length_m` |
| `FEMUR_LENGTH` | `femur_length_m` |
| `GAIT_LEG_WAVE_LEG_LIFT_HEIGHT` | `gait.leg_wave.leg_lift_height_m` |
| `GAIT_LOOK_BODY_MAX_YAW` | `gait.look.torso_max_yaw_deg` |
| `GAIT_LOOK_HEAD_MAX_YAW` | `gait.look.head_max_yaw_deg` |
| `GAIT_MOVE_COMBINED_HYSTERESIS_MARGIN` | `gait.move_combined.hysteresis_margin` |
| `GAIT_MOVE_COMBINED_MAX_VELOCITY_LINEAR` | `gait.move_combined.max_linear_velocity_m_s` |
| `GAIT_MOVE_COMBINED_MAX_VELOCITY_ROTATION` | `gait.move_combined.max_angular_velocity_rad_s` |
| `GAIT_MOVE_COMBINED_ROTATION_WEIGHT` | `gait.move_combined.rotation_weight` |
| `GAIT_MOVE_COMBINED_THRESHOLD_RIPPLE_TRIPOD` | `gait.move_combined.velocity_threshold_ripple_tripod` |
| `GAIT_MOVE_COMBINED_THRESHOLD_WAVE_RIPPLE` | `gait.move_combined.velocity_threshold_wave_ripple` |
| `GAIT_MOVE_COMBINED_TRANSITION_PHASE_LENGTH` | `gait.move_combined.transition_phase_span_rad` |
| `GAIT_MOVE_COMBINED_VELOCITY_FILTER_ALPHA` | `gait.move_combined.velocity_filter_alpha` |
| `GAIT_RIPPLE_FACTOR_VELOCITY_TO_CYCLE_TIME` | `gait.ripple.velocity_to_phase_gain` |
| `GAIT_RIPPLE_HEAD_MAX_YAW` | `gait.ripple.head_max_yaw_deg` |
| `GAIT_RUNNING_FACTOR_VELOCITY_TO_CYCLE_TIME` | `gait.running.velocity_to_phase_gain` |
| `GAIT_RUNNING_FLIGHT_FRACTION` | `gait.running.flight_fraction` |
| `GAIT_RUNNING_HEAD_MAX_YAW` | `gait.running.head_max_yaw_deg` |
| `GAIT_RUNNING_ROTATION_WEIGHT` | `gait.running.rotation_weight` |
| `GAIT_RUNNING_VELOCITY_FILTER_ALPHA` | `gait.running.velocity_filter_alpha` |
| `GAIT_TRIPOD_FACTOR_VELOCITY_TO_CYCLE_TIME` | `gait.tripod.velocity_to_phase_gain` |
| `GAIT_TRIPOD_HEAD_MAX_YAW` | `gait.tripod.head_max_yaw_deg` |
| `GAIT_WATCH_BODY_MAX_YAW` | `gait.watch.torso_max_yaw_deg` |
| `GAIT_WAVE_FACTOR_VELOCITY_TO_CYCLE_TIME` | `gait.wave.velocity_to_phase_gain` |
| `GAIT_WAVE_HEAD_MAX_YAW` | `gait.wave.head_max_yaw_deg` |
| `GENERIC_BODY_MAX_PITCH` | `gait.generic.torso_max_pitch_deg` |
| `GENERIC_BODY_MAX_ROLL` | `gait.generic.torso_max_roll_deg` |
| `GENERIC_HEAD_MAX_PITCH` | `gait.generic.head_max_pitch_deg` |
| `GENERIC_HEAD_MAX_YAW` | `gait.generic.head_max_yaw_deg` |
| `GENERIC_LEG_LIFT_HEIGHT` | `gait.generic.leg_lift_height_m` |
| `GENERIC_STEP_LENGTH` | `gait.generic.step_length_m` |
| `OFFSET_COXA_ANGLE_DEG` | `yaw_deg` |
| `SERIAL_PORT` | `servo.serial_port` |
| `SERVO_ADAPTATION_DEG` | `servo.adaptation_deg` |
| `SERVO_CONTROLLER_OFFLINE` | `servo.offline` |
| `SERVO_CONTROLLER_TYPE` | `servo.controller_type` |
| `SERVO_NAME` | `servo.names` |
| `SERVO_OFFSET_DEG` | `servo.offset_deg` |
| `SERVO_ORIENTATION_CLOCKWISE` | `servo.orientation_clockwise` |
| `SERVO_SERIAL_ID` | `servo.serial_ids` |
| `TESTLEGS_COXA_DELTA_DEG` | `gait.test_legs.torso_coxa_delta_deg` |
| `TESTLEGS_FEMUR_DELTA_DEG` | `gait.test_legs.coxa_femur_delta_deg` |
| `TESTLEGS_TIBIA_DELTA_DEG` | `gait.test_legs.femur_tibia_delta_deg` |
| `TIBIA_LENGTH` | `tibia_length_m` |
| `footPositions_standing` | `toe_positions_standing` |
| `footPositions_laydown` | `toe_positions_laydown` |

Legacy action-package YAML follows the same naming: `body` becomes `torso`,
`legAngles` becomes `leg_angles`, `footPositions` becomes `toe_positions`,
`factorDuration` becomes `duration_factor`, and `All` becomes `all`.
Leg-angle keys are `torso_coxa`, `coxa_femur`, and `femur_tibia`.
Shared ROS message fields, gait IDs, servo names/IDs, and URDF joint names
retain their existing protocol names.
