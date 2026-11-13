#!/usr/bin/env python3
"""Rock-and-roll choreography: headbangs, hip twists, six-foot fills and kick solos."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choreography import Stage, pose, positive

SONG = 'musicfox_excess_party.mp3'


def riff(stage, cycles=2):
    """Four-beat hip twist: counter-looking head and a low backbeat."""
    for _ in range(cycles):
        for side in (1, -1):
            stage.cue(head={'yaw': -side * 18, 'pitch': -8},
                      turn={'yaw': side * 7, 'roll': side * 4},
                      body={'y': side * 0.012, 'z': -0.006})
            stage.cue(head={'yaw': -side * 10, 'pitch': 12},
                      turn={'yaw': side * 4, 'pitch': 4},
                      body={'y': side * 0.006, 'z': -0.012})
    stage.cue()


def kick(stage, leg, side, *, reach=0.025):
    """Lift, kick while airborne, retract while airborne, then plant vertically.

    Only one foot is lifted. The other five retain their original positions.
    Body shift is away from the lifted foot; it persists until that foot lands.
    """
    front = leg.endswith('front')
    rear = leg.endswith('back')
    bias = {'x': -0.012 if front else (0.012 if rear else 0), 'y': -side * 0.01,
            'z': -0.006}
    stage.cue(0.5, body=bias, head={'yaw': side * 12})
    stage.cue(0.75, body=bias, head={'yaw': side * 18, 'pitch': -6},
              feet={leg: {'z': 0.025}})
    stage.cue(0.75, body=bias, head={'yaw': -side * 12, 'pitch': 10},
              turn={'yaw': -side * 4},
              feet={leg: {'x': reach if front else (-reach if rear else 0),
                          'y': side * (0.012 if front or rear else reach), 'z': 0.025}})
    stage.cue(0.75, body=bias, head={'yaw': side * 10, 'pitch': -5},
              feet={leg: {'z': 0.025}})
    stage.cue(0.75, body=bias, head={'pitch': 8})
    stage.cue(0.5)


def drum_fill(stage):
    """A travelling six-foot tom fill, alternating sides front to back."""
    for leg in ('right_front', 'left_front', 'right_mid', 'left_mid', 'right_back', 'left_back'):
        side = -1 if leg.startswith('right') else 1
        kick(stage, leg, side, reach=0.016)


def body_roll(stage):
    """A circular torso accent with feet planted and head counter-motion."""
    for roll, pitch in ((0, -5), (5, 0), (0, 5), (-5, 0)):
        stage.cue(head={'roll': -roll, 'pitch': -pitch * 2},
                  turn={'roll': roll, 'pitch': pitch}, body={'z': -0.008})
    stage.cue()


def make_stage(bpm=112, amplitude=1.0):
    positive(bpm, 'bpm')
    positive(amplitude, 'amplitude')
    if not 60 <= bpm <= 140 or not 0 < amplitude <= 1:
        raise ValueError('Use 60–140 bpm and amplitude greater than 0 and at most 1')
    stage = Stage(bpm, amplitude)
    with stage.section('intro'):
        # Entrance: capture the exact starting pose once, then crouch on the count-in.
        stage.cue(2, head={'pitch': -8}, body={'z': -0.01}, save=True)
        stage.cue(2, head={'pitch': 10}, turn={'pitch': 3}, body={'z': -0.006})
        stage.cue()
    with stage.section('verse'):
        riff(stage, 2)
    with stage.section('guitar_solo'):
        # Lead guitar: alternating front-foot kicks, looking against the kick.
        for _ in range(2):
            kick(stage, 'right_front', -1)
            kick(stage, 'left_front', 1)
    with stage.section('body_roll'):
        body_roll(stage)
    with stage.section('drum_fill'):
        drum_fill(stage)
    with stage.section('breakdown'):
        # Breakdown: headbang on the downbeat, rebound on the longer upbeat.
        for _ in range(4):
            stage.cue(0.75, head={'pitch': 14}, turn={'pitch': 5}, body={'z': -0.012})
            stage.cue(1.25, head={'pitch': -10}, turn={'pitch': -3}, body={'z': -0.003})
        stage.cue()
    with stage.section('encore'):
        # Encore: another twist riff and wide middle-foot side kicks.
        riff(stage, 2)
        kick(stage, 'right_mid', -1, reach=0.025)
        kick(stage, 'left_mid', 1, reach=0.025)
        body_roll(stage)
    with stage.section('finale'):
        # Final chord: all six feet planted, forward bow, hold, then recenter.
        stage.cue(2, head={'pitch': 14}, turn={'pitch': 6}, body={'x': 0.012, 'z': -0.012})
        stage.cue(2, head={'pitch': 14}, turn={'pitch': 6}, body={'x': 0.012, 'z': -0.012})
        stage.cue(2)
    # All authored deltas already sum to zero; restore removes numerical residuals.
    stage.actions.append(pose(2.0, restore=True))
    return stage


def build(bpm=112, amplitude=1.0, section=None):
    stage = make_stage(bpm, amplitude)
    sequence = stage.sequence('dance_rock', section)
    if section is None:
        sequence.actions[0]['music'] = {'song': SONG, 'background': True}
        finale_start = stage.sections['finale'][0]
        # Fade through the bow, hold, recenter and final entrance-pose restore.
        fade_s = sum(action['movement']['duration_s'] for action in sequence.actions[finale_start:])
        sequence.actions[finale_start]['music'] = {'song': 'STOP', 'fade_out_s': fade_s}
        sequence.extend([{'music': 'STOP'}])
    return sequence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bpm', type=float, default=112)
    parser.add_argument('--amplitude', type=float, default=1.0)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--list-sections', action='store_true')
    parser.add_argument('--section')
    parser.add_argument('--report', action='store_true')
    args = parser.parse_args()
    try:
        stage = make_stage(args.bpm, args.amplitude)
        if args.list_sections:
            print('\n'.join(stage.sections))
            return
        sequence = build(args.bpm, args.amplitude, args.section)
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
