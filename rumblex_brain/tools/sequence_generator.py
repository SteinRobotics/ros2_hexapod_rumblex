#!/usr/bin/env python3
"""Offline sequence authoring and GUI. Run without arguments to open the editor."""
import argparse
from contextlib import contextmanager
import copy
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import tempfile

import yaml

LEGS = ('right_front', 'right_mid', 'right_back', 'left_front', 'left_mid', 'left_back')
MINIMUM_ABSOLUTE_POSE_DURATION = 0.1

CURVES = ('linear', 'sin_squared', 'sin', 'cos')
GAITS = ('MOVE_COMBINED', 'MOVE_TRIPOD', 'MOVE_RIPPLE', 'MOVE_WAVE', 'MOVE_CRAWL')
BUILTINS = ('CONTINUOUS_POSE', 'SEQUENCE_STAND_UP', 'SEQUENCE_LAYDOWN', 'SEQUENCE_WAITING',
            'SEQUENCE_WATCH', 'SEQUENCE_DANCE', 'SEQUENCE_LEGS_WAVE', 'SEQUENCE_BODY_ROLL',
            'SEQUENCE_BITE', 'SEQUENCE_STOMP', 'SEQUENCE_TESTLEGS', 'HOLD_DISTANCE', 'LEVEL_TORSO')
PRESETS = ('square', 'circle', 'figure_eight', 'forward', 'curious_look', 'dance_rock', 'dance_metal')


def positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{label} must be a finite positive number')
    return value


def beats(count, bpm):
    """Convert beats to seconds; playback handoffs are not included."""
    return 60 * positive(count, 'beats') / positive(bpm, 'bpm')


def vector(value, axes, complete=False):
    if not isinstance(value, dict) or set(value) - set(axes) or (complete and set(value) != set(axes)):
        raise ValueError(f'Expected coordinates {axes}')
    for number in value.values():
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise ValueError('Coordinates must be finite numbers')


def legs(value, axes, complete=False):
    if not isinstance(value, dict) or not value or set(value) - set(LEGS):
        raise ValueError('Expected known leg names')
    if complete and set(value) != set(LEGS):
        raise ValueError('Absolute toes require all six legs')
    for target in value.values():
        vector(target, axes, complete=True)


def pose(duration_s, *, body_pose=None, pose_delta=None, joint_targets=None,
         save=False, restore=False, interpolation='sin_squared'):
    """Create one SINGLE_POSE action. Saving captures the pose before motion."""
    positive(duration_s, 'duration_s')
    if interpolation not in CURVES:
        raise ValueError('Unknown interpolation')
    if not isinstance(save, bool) or not isinstance(restore, bool):
        raise ValueError('save and restore must be booleans')
    if restore and (save or any(v is not None for v in (body_pose, pose_delta, joint_targets))):
        raise ValueError('Restore cannot have targets or save')
    if pose_delta is not None and (body_pose is not None or joint_targets is not None):
        raise ValueError('Relative targets cannot be combined with other targets')
    if not restore and all(v is None for v in (body_pose, pose_delta, joint_targets)):
        raise ValueError('A pose target is required')
    if (pose_delta is None or save or restore) and duration_s < MINIMUM_ABSOLUTE_POSE_DURATION:
        raise ValueError('Absolute, joint, save, and restore actions require at least 0.1 seconds')
    if body_pose is not None:
        allowed = {'head_orientation', 'torso_pose', 'toe_positions'}
        if not isinstance(body_pose, dict) or not body_pose or set(body_pose) - allowed:
            raise ValueError('Invalid body_pose components')
        if 'head_orientation' in body_pose:
            vector(body_pose['head_orientation'], ('roll', 'pitch', 'yaw'))
        if 'torso_pose' in body_pose:
            torso = body_pose['torso_pose']
            if not isinstance(torso, dict) or set(torso) - {'position', 'orientation'}:
                raise ValueError('Invalid torso_pose')
            if 'position' in torso:
                vector(torso['position'], ('x', 'y', 'z'))
            if 'orientation' in torso:
                vector(torso['orientation'], ('roll', 'pitch', 'yaw'))
        if 'toe_positions' in body_pose:
            if joint_targets is not None:
                raise ValueError('Joint targets conflict with toe positions')
            legs(body_pose['toe_positions'], ('x', 'y', 'z'), complete=True)
    if pose_delta is not None:
        allowed = {'head_rotation', 'torso_rotation', 'torso_translation', 'toe_translation', 'coxa_rotation_deg'}
        if not isinstance(pose_delta, dict) or not pose_delta or set(pose_delta) - allowed:
            raise ValueError('Invalid pose_delta fields')
        for key, value in pose_delta.items():
            if key in ('toe_translation', 'coxa_rotation_deg'):
                if not isinstance(value, dict) or not value or set(value) - set(LEGS):
                    raise ValueError('Expected known leg names')
                for target in value.values():
                    vector({'angle': target}, ('angle',)) if key == 'coxa_rotation_deg' else vector(target, ('x', 'y', 'z'))
            else:
                vector(value, ('roll', 'pitch', 'yaw') if key.endswith('rotation') else ('x', 'y', 'z'))
    if joint_targets is not None:
        legs(joint_targets, ('torso_coxa', 'coxa_femur', 'femur_tibia'))
    movement = dict(name='SINGLE_POSE', duration_s=duration_s, interpolation=interpolation)
    for key, value in (('body_pose', body_pose), ('pose_delta', pose_delta), ('joint_targets', joint_targets)):
        if value is not None:
            movement[key] = copy.deepcopy(value)
    if save:
        movement['save_pose'] = True
    if restore:
        movement['restore_pose'] = True
    return {'movement': movement}


def hold(duration_s):
    """Consume time with a zero relative motion, retaining the current target."""
    return pose(duration_s, pose_delta={'head_rotation': {'yaw': 0}})


def repeat(times, actions):
    if isinstance(times, bool) or not isinstance(times, int) or not 1 <= times <= 1000:
        raise ValueError('Repetition count must be an integer from 1 to 1000')
    if not actions:
        raise ValueError('A phrase cannot be empty')
    return {'loop': {'times': times, 'actions': copy.deepcopy(list(actions))}}


class Sequence:
    """Compose movement phrases; validate again before exporting mutable actions."""
    def __init__(self, name, *, trigger=None):
        if not isinstance(name, str) or not name.strip():
            raise ValueError('Sequence name is required')
        if trigger is not None and (not isinstance(trigger, dict) or set(trigger) - {'voice', 'joystick'} or
                                    any(not isinstance(v, str) or not v for v in trigger.values())):
            raise ValueError('Triggers must be nonempty voice/joystick strings')
        self.name = name
        self.trigger = copy.deepcopy(trigger)
        self.actions = []

    def extend(self, actions):
        self.actions.extend(copy.deepcopy(list(actions)))
        return self

    def summary(self):
        # Metadata is mutable in the editor, so validate it again on export.
        Sequence(self.name, trigger=self.trigger)
        saved, count, seconds = False, 0, 0.0

        def visit(actions, depth=0):
            nonlocal saved, count, seconds
            if not isinstance(actions, list) or not actions:
                raise ValueError('Actions must be a nonempty list')
            for action in actions:
                if not isinstance(action, dict):
                    raise ValueError('Expected action mapping')
                if set(action) == {'loop'}:
                    loop = action['loop']
                    if not isinstance(loop, dict) or set(loop) != {'times', 'actions'} or depth >= 16:
                        raise ValueError('Invalid loop or nesting exceeds 16')
                    repeat(loop['times'], loop['actions'])
                    for _ in range(loop['times']):
                        visit(loop['actions'], depth + 1)
                    continue
                duration = validate_action(action)
                movement = action.get('movement', {})
                save = movement.get('save_pose', False)
                restore = movement.get('restore_pose', False)
                if save:
                    if saved:
                        raise ValueError('Nested save is not supported')
                    saved = True
                if restore:
                    if not saved:
                        raise ValueError('Restore requires a saved pose')
                    saved = False
                count += 1
                seconds += duration
                if count > 10000:
                    raise ValueError('Sequence exceeds 10000 expanded actions')
        visit(self.actions)
        if saved:
            raise ValueError('Saved pose must be restored on normal completion')
        if not math.isfinite(seconds):
            raise ValueError('Total duration is not finite')
        return count, seconds

    def to_yaml(self, *, recipe=True):
        self.summary()
        document = {'name': self.name}
        if self.trigger:
            document['trigger'] = self.trigger
        document['actions'] = self.actions

        class Dumper(yaml.SafeDumper):
            def ignore_aliases(self, data):
                return True
        header = ('# Generated from a Python choreography recipe; edit the recipe and regenerate.\n'
                  '# Units: metres, degrees, seconds.\n' if recipe else
                  '# Authored with sequence_generator.py; editable runtime sequence.\n'
                  '# Units: metres, seconds; pose angles in degrees, angular velocity in rad/s.\n')
        return header + yaml.dump(document, Dumper=Dumper, sort_keys=False)

    def write(self, path, *, force=False, recipe=True):
        """Validate first, then atomically publish without truncating existing files."""
        content = self.to_yaml(recipe=recipe)
        path = Path(path)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                             prefix=f'.{path.name}.', suffix='.tmp', delete=False) as output:
                temporary = Path(output.name)
                output.write(content)
                output.flush()
                os.fsync(output.fileno())
            if force:
                if path.is_file():
                    temporary.chmod(path.stat().st_mode & 0o777)
                os.replace(temporary, path)
            else:
                # Linking is exclusive: an existing destination wins even in a race.
                os.link(temporary, path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    @classmethod
    def from_yaml(cls, content):
        """Load self-contained authoring YAML; never follow includes or imports."""
        document = load_yaml(content)
        if not isinstance(document, dict) or set(document) - {'name', 'trigger', 'actions', 'variables'}:
            raise ValueError('Expected name, trigger, actions and optional YAML-anchor variables; '
                             'includes/imports and runtime variable expressions are not supported')
        sequence = cls(document.get('name'), trigger=document.get('trigger'))
        sequence.actions = document.get('actions')
        sequence.summary()
        return sequence


class Stage:
    """Author offsets from the entrance pose; compile successive targets to deltas.

    Every cue specifies the complete offset, so omitted axes return to their
    entrance value. This is recipe bookkeeping, not a new runtime pose frame.
    """
    def __init__(self, bpm, amplitude):
        self.bpm = positive(bpm, 'bpm')
        self.amplitude = positive(amplitude, 'amplitude')
        self.state = {}
        self.actions = []
        self.sections = {}
        self._active_section = None

    def cue(self, length=1, *, head=None, turn=None, body=None, feet=None, save=False):
        return self._cue(length, head=head, turn=turn, body=body, feet=feet, save=save)

    def change(self, length=1, *, head=None, turn=None, body=None, feet=None):
        """Replace supplied axis offsets; retain all omitted axes."""
        return self._cue(length, head=head, turn=turn, body=body, feet=feet, partial=True)

    def _cue(self, length, *, head=None, turn=None, body=None, feet=None, save=False, partial=False):
        fields = dict(head_rotation=head, torso_rotation=turn,
                      torso_translation=body, toe_translation=feet)
        supplied = {key: value for key, value in fields.items() if value is not None}
        pose(1, pose_delta=supplied or {'head_rotation': {'yaw': 0}})
        target = dict(self.state) if partial else {}
        for field, values in (('head_rotation', head), ('torso_rotation', turn),
                              ('torso_translation', body)):
            for axis, value in (values or {}).items():
                target[(field, axis)] = value * self.amplitude
        for leg, values in (feet or {}).items():
            for axis, value in values.items():
                target[('toe_translation', leg, axis)] = value * self.amplitude
        if any(not math.isfinite(value) for value in target.values()):
            raise ValueError('Scaled target must be finite')
        delta = {}
        for key in sorted(self.state.keys() | target.keys()):
            difference = round(target.get(key, 0) - self.state.get(key, 0), 9)
            if difference:
                branch = delta
                for part in key[:-1]:
                    branch = branch.setdefault(part, {})
                branch[key[-1]] = difference
        if not delta:
            delta = {'head_rotation': {'yaw': 0}}
        duration = beats(length, self.bpm)
        if save:
            duration = max(MINIMUM_ABSOLUTE_POSE_DURATION, duration)
        self.actions.append(pose(duration, pose_delta=delta, save=save))
        self.state = target

    @contextmanager
    def section(self, name):
        """Mark a unique, non-nested range. Failed sections roll back their cues."""
        if self._active_section is not None or not isinstance(name, str) or not name.strip() or name in self.sections:
            raise ValueError('Sections require unique nonempty names and cannot nest')
        start, state = len(self.actions), dict(self.state)
        self._active_section = name
        try:
            yield self
            if len(self.actions) == start:
                raise ValueError('Section cannot be empty')
            self.sections[name] = (start, len(self.actions), state, dict(self.state))
        except Exception:
            del self.actions[start:]
            self.state = state
            raise
        finally:
            self._active_section = None

    def sequence(self, name, section=None):
        if self._active_section is not None:
            raise ValueError('Finish the active section before exporting')
        if section is None:
            return Sequence(name).extend(self.actions)
        if section not in self.sections:
            raise ValueError(f'Unknown section: {section}')
        start, end, before, after = self.sections[section]
        if any(abs(value) > 1e-6 for state in (before, after) for value in state.values()):
            raise ValueError('Rehearsal requires zero entrance/exit offsets and planted feet')
        actions = copy.deepcopy(self.actions[start:end])
        for action in actions:
            movement = action['movement']
            if movement.get('restore_pose'):
                raise ValueError('Section restores must be outside the authored section')
            movement.pop('save_pose', None)
        entrance = pose(1, save=True, pose_delta={'head_rotation': {'yaw': 0}})
        return Sequence(f'{name}_{section}').extend([entrance, *actions, pose(2, restore=True)])

    def report(self, section=None):
        """Report inferred contact, assuming all feet start on level ground."""
        if section is not None:
            actions = self.sequence('rehearsal', section).actions
            ranges = [('entrance', 0, 1), (section, 1, len(actions) - 1),
                      ('return', len(actions) - 1, len(actions))]
        else:
            actions = self.actions
            ranges = [(name, entry[0], entry[1]) for name, entry in self.sections.items()]
        elapsed = [0.0]
        state = {leg: dict(x=0.0, y=0.0, z=0.0) for leg in LEGS}
        saved = None
        rows, warnings = [], []
        minimum = 6
        tolerance = 1e-6
        for index, action in enumerate(actions):
            movement = action['movement']
            previous = copy.deepcopy(state)
            if movement.get('save_pose'):
                saved = copy.deepcopy(state)
            if movement.get('restore_pose'):
                if saved is None:
                    raise ValueError('Restore requires a saved pose')
                state, saved = saved, None
            else:
                if 'pose_delta' not in movement:
                    raise ValueError('Contact reporting requires Stage relative movements')
                for leg, coordinates in movement['pose_delta'].get('toe_translation', {}).items():
                    for axis, value in coordinates.items():
                        state[leg][axis] += value
            phases = {}
            for leg in LEGS:
                before, after = previous[leg], state[leg]
                up_before, up_after = before['z'] > tolerance, after['z'] > tolerance
                phases[leg] = ('swing' if up_before and up_after else 'lift' if up_after
                               else 'landing' if up_before else 'support')
                if min(before['z'], after['z']) < -tolerance:
                    warnings.append(f'action {index + 1}: {leg} below entrance ground height')
                horizontal = any(abs(after[axis] - before[axis]) > tolerance for axis in ('x', 'y'))
                if horizontal and not (up_before and up_after):
                    warnings.append(f'action {index + 1}: {leg} horizontal movement during ground contact')
            minimum = min(minimum, sum(phase == 'support' for phase in phases.values()))
            duration = movement['duration_s']
            rows.append(f'{index + 1:3d} {elapsed[-1]:7.2f}–{elapsed[-1] + duration:7.2f} s  ' +
                        ', '.join(f'{leg}={phase}' for leg, phase in phases.items()))
            elapsed.append(elapsed[-1] + duration)
        lines = ['Motion timing excludes scheduler delays.',
                 'Inferred contact assumes a level-standing entrance; not measured contact or balance.',
                 'Sections:']
        for name, start, end in ranges:
            lines.append(f'  {name}: {elapsed[start]:.2f}–{elapsed[end]:.2f} s; '
                         f'{elapsed[end] - elapsed[start]:.2f} s; {end - start} actions')
        lines.extend([f'Total: {elapsed[-1]:.2f} s; {len(actions)} actions',
                      f'Minimum commanded support feet: {minimum}', 'Contacts by action:', *rows])
        lines.extend(['Warnings:', *(warnings or ['  none'])])
        return '\n'.join(lines)


def finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return value


def keys(value, allowed, label):
    if not isinstance(value, dict) or set(value) - set(allowed):
        raise ValueError(f'{label}: expected fields {", ".join(allowed)}')


def nonnegative(value, label):
    if finite(value, label) < 0:
        raise ValueError(f'{label} must not be negative')
    return value


def move(duration_s, *, x=0.0, y=0.0, yaw=0.0, gait='MOVE_COMBINED'):
    """Timed body-frame velocity: x forward, y left (m/s), yaw left (rad/s).

    Every segment specifies every velocity axis, so preceding commands cannot leak.
    Finish a route with stop(); durations alone do not clear runtime velocity.
    """
    positive(duration_s, 'duration_s')
    if gait not in GAITS:
        raise ValueError(f'Choose a walking gait from {GAITS}')
    for value, label in ((x, 'x'), (y, 'y'), (yaw, 'yaw')):
        finite(value, label)
    return {'movement': {'name': gait, 'duration_s': duration_s},
            'velocity': {'linear': {'x': x, 'y': y, 'z': 0.0},
                         'angular': {'x': 0.0, 'y': 0.0, 'z': yaw}}}


def travel(distance_m, speed_m_s=0.03, *, bearing_deg=0.0, gait='MOVE_COMBINED'):
    """Travel a signed nominal distance along a body-frame bearing (degrees)."""
    finite(distance_m, 'distance_m')
    finite(bearing_deg, 'bearing_deg')
    positive(abs(distance_m), 'absolute distance_m')
    positive(speed_m_s, 'speed_m_s')
    speed = math.copysign(speed_m_s, distance_m)
    bearing = math.radians(bearing_deg)
    return move(abs(distance_m) / speed_m_s, x=speed * math.cos(bearing),
                y=speed * math.sin(bearing), gait=gait)


def turn(angle_deg, yaw_rate=0.2, *, gait='MOVE_COMBINED'):
    """Turn through a signed nominal angle; positive means counterclockwise."""
    finite(angle_deg, 'angle_deg')
    positive(abs(angle_deg), 'absolute angle_deg')
    positive(yaw_rate, 'yaw_rate')
    return move(abs(math.radians(angle_deg)) / yaw_rate,
                yaw=math.copysign(yaw_rate, angle_deg), gait=gait)


def arc(radius_m, angle_deg=360, speed_m_s=0.03, *, gait='MOVE_COMBINED'):
    """Drive forwards around a nominal radius. Negative angles curve right."""
    positive(radius_m, 'radius_m')
    positive(speed_m_s, 'speed_m_s')
    finite(angle_deg, 'angle_deg')
    positive(abs(angle_deg), 'absolute angle_deg')
    rate = speed_m_s / radius_m
    return move(abs(math.radians(angle_deg)) / rate, x=speed_m_s,
                yaw=math.copysign(rate, angle_deg), gait=gait)


def builtin(name, duration_s=1.5, *, distance_m=None):
    if name not in BUILTINS:
        raise ValueError('Unknown built-in movement')
    positive(duration_s, 'duration_s')
    movement = {'name': name, 'duration_s': duration_s}
    if name == 'HOLD_DISTANCE':
        movement['distance_m'] = positive(distance_m, 'distance_m')
    elif distance_m is not None:
        raise ValueError('distance_m requires HOLD_DISTANCE')
    return {'movement': movement}


def stop(duration_s=0.1):
    """Switch to pose holding and explicitly clear all velocity axes."""
    action = move(duration_s)
    action['movement']['name'] = 'CONTINUOUS_POSE'
    return action


def validate_action(action):
    """Validate the editor's supported runtime subset and return nominal seconds."""
    keys(action, ('movement', 'velocity', 'say', 'music'), 'Action')
    if not action:
        raise ValueError('Action cannot be empty')
    durations = [0.0]
    movement = action.get('movement')
    if movement is not None:
        keys(movement, ('name', 'duration_s', 'minDuration', 'direction', 'distance_m', 'interpolation',
                        'body_pose', 'pose_delta', 'joint_targets', 'save_pose', 'restore_pose'), 'Movement')
        name = movement.get('name')
        if name == 'SINGLE_POSE':
            options = dict(movement)
            options.pop('name')
            minimum = nonnegative(options.pop('minDuration', 0), 'minDuration')
            save, restore = options.pop('save_pose', False), options.pop('restore_pose', False)
            try:
                pose(**options, save=save, restore=restore)
            except TypeError as error:
                raise ValueError(f'Invalid pose fields: {error}') from error
            durations.append(max(minimum, movement['duration_s']))
        else:
            keys(movement, ('name', 'duration_s', 'minDuration', 'direction', 'distance_m'), 'Gait')
            if name not in (*GAITS, *BUILTINS):
                raise ValueError(f'Unsupported movement: {name}')
            duration = movement.get('duration_s', 1.0)
            positive(duration, 'duration_s')
            if name == 'HOLD_DISTANCE':
                positive(movement.get('distance_m'), 'distance_m')
            elif 'distance_m' in movement:
                raise ValueError('distance_m requires HOLD_DISTANCE')
            if 'direction' in movement and movement['direction'] not in ('CLOCKWISE', 'ANTICLOCKWISE'):
                raise ValueError('Unknown direction')
            if name in ('HOLD_DISTANCE', 'LEVEL_TORSO') and 'velocity' in action:
                raise ValueError('Sensor gaits own their velocity')
            durations.append(max(duration, nonnegative(movement.get('minDuration', 0), 'minDuration')))
    elif 'movement' in action:
        raise ValueError('Movement must be a mapping')
    if 'velocity' in action:
        velocity = action['velocity']
        keys(velocity, ('linear', 'angular', 'minDuration'), 'Velocity')
        for field in ('linear', 'angular'):
            if field in velocity:
                vector(velocity[field], ('x', 'y', 'z'))
        durations.append(nonnegative(velocity.get('minDuration', 0), 'minDuration'))
    for field in ('say', 'music'):
        if field not in action:
            continue
        value = action[field]
        if isinstance(value, str):
            continue
        allowed = ('text', 'language', 'minDuration') if field == 'say' else (
            'song', 'volume', 'background', 'fade_out_s', 'minDuration')
        keys(value, allowed, field)
        for key in (('text', 'language') if field == 'say' else ('song',)):
            if key in value and not isinstance(value[key], str):
                raise ValueError(f'{key} must be text')
        if 'fade_out_s' in value:
            fade = nonnegative(value['fade_out_s'], 'fade_out_s')
            if fade > 60 or (fade > 0 and value.get('song') != 'STOP'):
                raise ValueError('Music fade_out_s must be 0-60 seconds and requires song: STOP')
        if 'background' in value and not isinstance(value['background'], bool):
            raise ValueError('background must be boolean')
        if 'volume' in value and not 0 <= finite(value['volume'], 'volume') <= 1:
            raise ValueError('volume must be between 0 and 1')
        durations.append(nonnegative(value.get('minDuration', 0), 'minDuration'))
    return max(durations)


def load_yaml(content):
    """Safe YAML with duplicate-key, recursion and size checks for editor imports."""
    if len(content.encode('utf-8')) > 2_000_000:
        raise ValueError('YAML input exceeds 2 MB')

    class Loader(yaml.SafeLoader):
        def construct_mapping(self, node, deep=False):
            self.flatten_mapping(node)
            result = {}
            for key_node, value_node in node.value:
                key = self.construct_object(key_node, deep=deep)
                if not isinstance(key, str) or key in result:
                    raise ValueError('YAML keys must be unique strings')
                result[key] = self.construct_object(value_node, deep=deep)
            return result

    try:
        document = yaml.load(content, Loader=Loader)
        budget = 100000

        def check(value, ancestors=(), depth=0):
            nonlocal budget
            budget -= 1
            if budget < 0 or depth > 64:
                raise ValueError('YAML structure is too large or deeply nested')
            if isinstance(value, str) and value.startswith('${') and value.endswith('}'):
                raise ValueError('Runtime variable expressions are not supported; use literal values or YAML anchors')
            if isinstance(value, (list, dict)):
                if id(value) in ancestors:
                    raise ValueError('Recursive YAML aliases are not supported')
                for child in (value.values() if isinstance(value, dict) else value):
                    check(child, (*ancestors, id(value)), depth + 1)
        check(document)
        return document
    except (yaml.YAMLError, RecursionError) as error:
        raise ValueError(f'Invalid YAML: {error}') from error


def expanded_actions(actions):
    """Iterate actions of an already validated sequence without copying loops."""
    for action in actions:
        if 'loop' in action:
            for _ in range(action['loop']['times']):
                yield from expanded_actions(action['loop']['actions'])
        else:
            yield action


@dataclass(frozen=True)
class RoutePoint:
    seconds: float
    x: float
    y: float
    yaw: float
    action: int


def integrate(x, y, heading, vx, vy, yaw_rate, duration):
    """Exact constant body-twist integration, stable for small angular velocities."""
    angle = yaw_rate * duration
    half = angle / 2
    sinc = math.sin(half) / half if abs(half) > 1e-9 else 1.0
    scale = duration * sinc
    mid = heading + half
    return (x + scale * (vx * math.cos(mid) - vy * math.sin(mid)),
            y + scale * (vx * math.sin(mid) + vy * math.cos(mid)), heading + angle)


def route_preview(sequence):
    """Nominal planar route; deliberately excludes pose changes and sensor feedback.

    Samples are bounded independently of authored duration. Velocity persists across
    groups exactly as a command does, while only walking gaits integrate travel.
    """
    sequence.summary()
    result = [RoutePoint(0.0, 0.0, 0.0, 0.0, -1)]
    vx = vy = rate = 0.0
    gait = 'CONTINUOUS_POSE'
    for index, action in enumerate(expanded_actions(sequence.actions)):
        duration = validate_action(action)
        if 'movement' in action:
            gait = action['movement']['name']
        if 'velocity' in action:
            velocity = action['velocity']
            vx = velocity.get('linear', {}).get('x', 0.0)
            vy = velocity.get('linear', {}).get('y', 0.0)
            rate = velocity.get('angular', {}).get('z', 0.0)
        moving = gait in GAITS
        # At most 24 samples per action; sufficient to distinguish circular paths.
        samples = 24 if moving and rate else 1
        start = result[-1]
        for step in range(1, samples + 1):
            elapsed = duration * step / samples
            x, y, yaw = integrate(start.x, start.y, start.yaw,
                                  vx if moving else 0, vy if moving else 0,
                                  rate if moving else 0, elapsed)
            if not all(math.isfinite(v) for v in (x, y, yaw, start.seconds + elapsed)):
                raise ValueError('Preview exceeds finite coordinate range')
            result.append(RoutePoint(start.seconds + elapsed, x, y, yaw, index))
    return result


def sequence_notes(sequence):
    """Authoring guidance based on actual commands, separate from schema validity."""
    sequence.summary()
    actions = list(expanded_actions(sequence.actions))
    notes = []
    if any(a.get('movement', {}).get('name') in GAITS for a in actions):
        notes.append('Route is open-loop: speed limits, gait transitions and slip change distance and heading.')
        last = next((a for a in reversed(actions) if 'movement' in a or 'velocity' in a), {})
        velocity = last.get('velocity', {})
        if last.get('movement', {}).get('name') != 'CONTINUOUS_POSE' or not velocity or any(
                n != 0 for field in ('linear', 'angular') for n in velocity.get(field, {}).values()):
            notes.append('Add Stop at the end: timed walking does not clear the velocity command.')
    if any(a.get('movement', {}).get('name') in ('HOLD_DISTANCE', 'LEVEL_TORSO') for a in actions):
        notes.append('Sensor-driven travel is unknown and is not drawn in the route preview.')
    if any('say' in a or 'music' in a for a in actions):
        notes.append('Audio completion can extend the nominal timeline.')
    if any(a.get('movement', {}).get('name') == 'SINGLE_POSE' for a in actions):
        notes.append('Pose targets are not simulated; preview does not check reachability or balance.')
    return notes


def preset(name, *, size=0.5, speed=0.03, yaw_rate=0.2, gait='MOVE_COMBINED',
           clockwise=False, bpm=112, amplitude=0.6):
    """Create editable routes or reuse the existing choreography recipes."""
    if name not in PRESETS:
        raise ValueError(f'Unknown preset: {name}')
    if name == 'curious_look':
        from examples.curious_look import build
        return build(amplitude=amplitude)
    if name in ('dance_rock', 'dance_metal'):
        if name == 'dance_rock':
            from examples.dance_rock import build
        else:
            from examples.dance_metal import build
        return build(bpm=bpm, amplitude=amplitude)
    positive(size, 'size')
    positive(speed, 'speed')
    positive(yaw_rate, 'yaw_rate')
    sign = -1 if clockwise else 1
    if name == 'square':
        actions = [repeat(4, [travel(size, speed, gait=gait), turn(sign * 90, yaw_rate, gait=gait)])]
    elif name == 'forward':
        actions = [travel(size, speed, gait=gait)]
    else:
        actions = [arc(size, sign * 360, speed, gait=gait)]
        if name == 'figure_eight':
            actions.append(arc(size, -sign * 360, speed, gait=gait))
    return Sequence(name).extend([builtin('SEQUENCE_STAND_UP'), *actions, stop()])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Open editor, optionally with --preset or --input')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--preset', choices=PRESETS)
    source.add_argument('--input', type=Path, help='Open/validate self-contained sequence YAML')
    parser.add_argument('--output', type=Path, help='Write YAML instead of stdout (CLI only)')
    parser.add_argument('--force', action='store_true', help='Allow overwriting --output')
    parser.add_argument('--name', help='Override sequence name')
    parser.add_argument('--size', type=float, default=0.5, help='Route side/distance/radius in metres')
    parser.add_argument('--speed', type=float, default=0.03, help='Linear speed in m/s')
    parser.add_argument('--yaw-rate', type=float, default=0.2, help='Square turn speed in rad/s')
    parser.add_argument('--gait', choices=GAITS, default=GAITS[0])
    parser.add_argument('--clockwise', action='store_true')
    parser.add_argument('--bpm', type=float, default=112)
    parser.add_argument('--amplitude', type=float, default=0.6)
    args = parser.parse_args(argv)
    gui = args.gui or (args.preset is None and args.input is None and args.output is None)
    if args.output and not (args.preset or args.input):
        parser.error('--output requires --preset or --input')
    if args.gui and args.output:
        parser.error('--output is for CLI generation; use Export YAML in the GUI')
    try:
        sequence = (Sequence.from_yaml(args.input.read_text(encoding='utf-8')) if args.input else
                    preset(args.preset or 'square', size=args.size, speed=args.speed, yaw_rate=args.yaw_rate,
                           gait=args.gait, clockwise=args.clockwise, bpm=args.bpm, amplitude=args.amplitude))
        if args.name is not None:
            sequence.name = args.name
        sequence.summary()
        if gui:
            try:
                from sequence_generator_gui import launch
                launch(sequence, args.input)
            except ImportError as error:
                raise ValueError('GUI requires Tkinter (install python3-tk); CLI requires only PyYAML') from error
        else:
            if args.output:
                sequence.write(args.output, force=args.force, recipe=False)
            else:
                print(sequence.to_yaml(recipe=False), end='')
            count, seconds = sequence.summary()
            print(f'{count} expanded actions; {seconds:.2f} s nominal (excluding handoffs)', file=sys.stderr)
            for note in sequence_notes(sequence):
                print(note, file=sys.stderr)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    # Recipes importing the API during CLI execution must share these classes.
    sys.modules.setdefault('sequence_generator', sys.modules[__name__])
    main()
