#!/usr/bin/env python3
"""Generate a planted-foot curious gesture, starting from a standing pose."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choreography import Sequence, hold, pose, positive, repeat


def curious_phrase(amplitude=1.0, direction='left', pause_s=1.2):
    positive(amplitude, 'amplitude')
    positive(pause_s, 'pause_s')
    if direction not in ('left', 'right'):
        raise ValueError('Direction must be left or right')
    side = 1 if direction == 'left' else -1
    yaw = side * (16 if side == 1 else 13) * amplitude
    return [
        pose(1.0, save=True, pose_delta={'head_rotation': {'pitch': -3 * amplitude}}),
        pose(1.2, pose_delta={'head_rotation': {'yaw': yaw}}),
        pose(1.4, pose_delta={'torso_rotation': {'yaw': yaw * 0.35}}),
        pose(1.0, pose_delta={'head_rotation': {'roll': side * 5 * amplitude, 'pitch': 2 * amplitude}}),
        hold(pause_s * (1 if side == 1 else 1.15)),
        pose(1.8, restore=True),
    ]


def build(amplitude=1.0, direction='left', pause_s=1.2, repetitions=2):
    other = 'right' if direction == 'left' else 'left'
    phrase = curious_phrase(amplitude, direction, pause_s) + curious_phrase(amplitude, other, pause_s)
    return Sequence('curious_look').extend([repeat(repetitions, phrase)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--amplitude', type=float, default=1.0)
    parser.add_argument('--direction', choices=('left', 'right'), default='left')
    parser.add_argument('--pause-s', type=float, default=1.2)
    parser.add_argument('--repetitions', type=int, default=2)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    try:
        sequence = build(args.amplitude, args.direction, args.pause_s, args.repetitions)
        if args.output:
            sequence.write(args.output, force=args.force)
        else:
            print(sequence.to_yaml(), end='')
        count, seconds = sequence.summary()
        print(f'{count} expanded actions; {seconds:.2f} s motion (excluding scheduler delays)', file=sys.stderr)
    except (ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
