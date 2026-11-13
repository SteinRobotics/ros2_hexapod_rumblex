#!/usr/bin/env python3
"""An in-place metal performance: stalking, chugs, hammer feet and blast beats."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choreography import Stage, pose, positive


def chug(stage, side):
    """Uneven three-hit riff: short punches separated by heavier rebounds."""
    for length, x, z, pitch in ((0.5, 0.012, -0.012, 12), (0.75, -0.006, -0.004, -7),
                                (0.5, 0.014, -0.014, 15), (0.75, -0.008, -0.005, -8),
                                (0.5, 0.01, -0.012, 12), (1.0, 0, -0.003, -3)):
        stage.cue(length, head={'pitch': pitch, 'yaw': -side * 10},
                  turn={'pitch': pitch * 0.25, 'yaw': side * 5},
                  body={'x': x, 'y': side * 0.006, 'z': z})
    stage.cue(1)


def hammer(stage, leg):
    """A single-foot hammer with an airborne hesitation before the downbeat.

    Shift first, lift vertically, feint down without touching, lift again, land.
    There is no below-ground target or intended impact against the floor.
    """
    side = -1 if leg.startswith('right') else 1
    x = -0.012 if leg.endswith('front') else (0.012 if leg.endswith('back') else 0)
    bias = {'x': x, 'y': -side * 0.01, 'z': -0.006}
    stage.cue(0.75, body=bias, head={'yaw': side * 14, 'pitch': -5})
    stage.cue(0.75, body=bias, head={'yaw': side * 14, 'pitch': -8}, feet={leg: {'z': 0.025}})
    stage.cue(0.5, body=bias, head={'yaw': -side * 8, 'pitch': 6}, feet={leg: {'z': 0.01}})
    stage.cue(0.5, body=bias, head={'yaw': side * 8, 'pitch': -4}, feet={leg: {'z': 0.022}})
    stage.cue(0.75, body=bias, head={'pitch': 12})
    stage.cue(0.75)


def windmill(stage, reverse=False):
    """Eight-point head/shoulder circle with all feet planted."""
    ring = [(0, -6), (4, -4), (6, 0), (4, 4), (0, 6), (-4, 4), (-6, 0), (-4, -4)]
    if reverse:
        ring.reverse()
    for roll, pitch in ring:
        stage.cue(0.75, turn={'roll': roll, 'pitch': pitch},
                  head={'roll': -roll * 0.8, 'pitch': -pitch * 2, 'yaw': roll * 2},
                  body={'y': roll * 0.001, 'z': -0.008})
    stage.cue(1)


def rake(stage, leg):
    """Side-foot air rake: out, sweep rearward, retract, then land vertically."""
    side = -1 if leg.startswith('right') else 1
    bias = {'y': -side * 0.012, 'z': -0.007}
    stage.cue(0.75, body=bias, head={'yaw': side * 16})
    stage.cue(0.75, body=bias, feet={leg: {'z': 0.022}})
    stage.cue(0.75, body=bias, turn={'yaw': -side * 4}, head={'yaw': -side * 12},
              feet={leg: {'x': 0.012, 'y': side * 0.018, 'z': 0.022}})
    stage.cue(0.75, body=bias, turn={'yaw': side * 4}, head={'yaw': side * 12},
              feet={leg: {'x': -0.012, 'y': side * 0.018, 'z': 0.022}})
    stage.cue(0.75, body=bias, feet={leg: {'z': 0.022}})
    stage.cue(0.75, body=bias, head={'pitch': 8})
    stage.cue(0.5)


def make_stage(bpm=120, amplitude=1.0):
    positive(bpm, 'bpm')
    positive(amplitude, 'amplitude')
    if not 60 <= bpm <= 140 or not 0 < amplitude <= 1:
        raise ValueError('Use 60–140 bpm and amplitude greater than 0 and at most 1')
    stage = Stage(bpm, amplitude)
    with stage.section('predator_intro'):
        stage.cue(3, save=True, body={'x': -0.008, 'z': -0.014}, head={'pitch': -8})
        for side in (1, -1):
            stage.cue(1, body={'x': -0.004, 'y': side * 0.008, 'z': -0.012},
                      head={'yaw': side * 18, 'pitch': -6}, turn={'yaw': side * 4})
        stage.cue(1, head={'pitch': 14}, turn={'pitch': 4}, body={'x': 0.012, 'z': -0.01})
        stage.cue(2)
    with stage.section('palm_muted_chugs'):
        for side in (1, -1, 1, -1):
            chug(stage, side)
    with stage.section('hammer_feet'):
        for leg in ('right_front', 'left_back', 'left_front', 'right_back'):
            hammer(stage, leg)
    with stage.section('windmill'):
        windmill(stage)
        windmill(stage, reverse=True)
    with stage.section('air_rakes'):
        rake(stage, 'right_mid')
        rake(stage, 'left_mid')
    with stage.section('stalking_breakdown'):
        # Two slow prowls, punctuated by a short planted-foot lunge and recoil.
        for side in (1, -1):
            stage.cue(2, head={'yaw': side * 18, 'pitch': -8},
                      turn={'yaw': side * 6}, body={'x': -0.008, 'y': side * 0.01, 'z': -0.014})
            stage.change(1, head={'yaw': 0})
            stage.cue(0.5, head={'pitch': 14}, turn={'pitch': 4}, body={'x': 0.016, 'z': -0.01})
            stage.cue(1, head={'pitch': -5}, body={'x': -0.006, 'z': -0.006})
            stage.cue(1.5)
    with stage.section('blast_beats'):
        # Fast head accents over a smaller torso pulse; feet stay planted.
        for side in (1, -1, 1, -1):
            stage.cue(0.5, head={'pitch': 12, 'yaw': side * 6},
                      turn={'pitch': 2}, body={'z': -0.008})
            stage.cue(0.5, head={'pitch': -8, 'yaw': side * 3}, body={'z': -0.004})
            stage.cue(0.5, head={'pitch': 10, 'yaw': -side * 6},
                      turn={'pitch': 2}, body={'z': -0.008})
            stage.cue(0.5, head={'pitch': -6}, body={'z': -0.004})
        stage.cue(2)
    with stage.section('final_crash'):
        stage.cue(1, head={'pitch': -8}, body={'x': -0.008, 'z': -0.012})
        stage.cue(0.75, head={'pitch': 16}, turn={'pitch': 6}, body={'x': 0.016, 'z': -0.014})
        stage.change(1.25)  # Freeze on the final chord.
        stage.cue(2, head={'pitch': 4}, body={'z': -0.004})
        stage.cue(2)
    stage.actions.append(pose(2, restore=True))
    return stage


def build(bpm=120, amplitude=1.0, section=None):
    return make_stage(bpm, amplitude).sequence('dance_metal', section)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bpm', type=float, default=120)
    parser.add_argument('--amplitude', type=float, default=1.0)
    parser.add_argument('--section')
    parser.add_argument('--list-sections', action='store_true')
    parser.add_argument('--report', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    try:
        stage = make_stage(args.bpm, args.amplitude)
        if args.list_sections:
            print('\n'.join(stage.sections))
            return
        sequence = stage.sequence('dance_metal', args.section)
        if args.report:
            print(stage.report(args.section), file=sys.stderr)
        if args.output:
            sequence.write(args.output, force=args.force)
        else:
            print(sequence.to_yaml(), end='')
        count, seconds = sequence.summary()
        print(f'{count} actions; {seconds:.2f} s motion (excluding scheduler delays)', file=sys.stderr)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
