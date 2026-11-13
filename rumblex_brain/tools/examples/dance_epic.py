#!/usr/bin/env python3
"""A 30-second opening performance for Iron Star March, starting standing."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from choreography import Sequence, Stage, pose

SONG = 'lovemusic_ai__Iron_Star_March.mp3'
DURATION_S = 30.0
FADE_START_S = 26.0
FADE_DURATION_S = DURATION_S - FADE_START_S


def make_stage():
    # Seconds-based clock, not a fixed BPM estimate. Phrase boundaries retain
    # measured opening accents; short subdivisions supply the fast footwork.
    stage = Stage(60, 1)
    elapsed = 0.0

    def at(time, **target):
        nonlocal elapsed
        time = round(time, 6)
        stage.cue(round(time - elapsed, 6), save=not stage.actions, **target)
        elapsed = time

    def tap(end, leg, strength=1):
        start = elapsed
        side = -1 if leg.startswith('right') else 1
        x = -0.004 if leg.endswith('front') else (0.004 if leg.endswith('back') else 0)
        for phase in range(4):
            time = start + (end - start) * (phase + 1) / 4
            # Scale targets, not successive deltas, to avoid cumulative drift.
            gain = strength * min(1, max(0, (DURATION_S - time) / FADE_DURATION_S))
            if phase == 3:
                at(time)
                continue
            bias = {'x': x * gain, 'y': -side * 0.005 * gain, 'z': -0.003 * gain}
            # Shift, lift vertically, land vertically, recenter. Never slide a
            # planted toe or lift the next foot before this one has landed.
            at(time, body=bias,
               head={'yaw': side * 5 * gain, 'pitch': (-3 if phase == 1 else 3) * gain},
               turn={'roll': side * 1.5 * gain},
               feet={leg: {'z': 0.010 * gain}} if phase == 1 else None)

    def phrase(name, end, legs, strength=1):
        start = elapsed
        with stage.section(name):
            for index, leg in enumerate(legs):
                tap(start + (end - start) * (index + 1) / len(legs), leg, strength)

    fronts = ('right_front', 'left_front')
    circuit = ('right_front', 'left_back', 'right_mid', 'left_front', 'right_back', 'left_mid')
    phrase('pickup_taps', 1.83, fronts, 0.6)
    phrase('front_patter', 5.13, fronts * 2, 0.8)
    phrase('six_foot_ripple', 11.62, circuit + fronts + ('right_mid',))
    phrase('driving_steps', 22.48, circuit * 2 + fronts * 2)
    phrase('fast_fill', FADE_START_S, circuit)
    phrase('fade_steps', 29.0, fronts * 2)
    with stage.section('return'):
        stage.actions.append(pose(DURATION_S - elapsed, restore=True))
    return stage


def build():
    stage = make_stage()
    stage.actions[0]['music'] = {'song': SONG, 'background': True}
    fade_index = stage.sections['fade_steps'][0]
    stage.actions[fade_index]['music'] = {'song': 'STOP', 'fade_out_s': FADE_DURATION_S}
    return Sequence('dance_epic').extend([*stage.actions, {'music': 'STOP'}])


def to_yaml():
    return ('# Recipe: tools/examples/dance_epic.py; regenerate after editing the recipe.\n'
            '# 30.00 s nominal including return; start standing.\n'
            '# Fast alternating taps and six-foot ripples; one foot airborne at a time.\n'
            '# Phrase anchors follow opening audio accents; subdivisions are authored.\n'
            '# 26-30 s: shrink movement targets and fade background music to silence.\n'
            '# Scheduler/audio-start latency is not compensated; final STOP is a fallback.\n' + build().to_yaml())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--report', action='store_true')
    args = parser.parse_args()
    content = to_yaml()
    if args.output:
        with args.output.open('w' if args.force else 'x', encoding='utf-8') as output:
            output.write(content)
    else:
        print(content, end='')
    if args.report:
        print(make_stage().report(), file=sys.stderr)


if __name__ == '__main__':
    main()
