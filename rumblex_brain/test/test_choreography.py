"""Offline authoring regression tests; no ROS runtime required."""
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'tools'))
from choreography import Stage, Sequence, beats, hold, pose, repeat
from examples.curious_look import build
from examples.dance_rock import build as build_rock, make_stage
from examples import dance_hot_dogs_for_breakfast as hot_dogs
from examples import dance_metal as metal
from examples import dance_epic as epic


class ChoreographyTest(unittest.TestCase):
    def test_epic_opening_timing_contacts_and_reproducibility(self):
        self.assertEqual(epic.to_yaml(), (PACKAGE / 'config/sequences/dance_epic.yaml').read_text())
        sequence = epic.build()
        self.assertEqual(sequence.name, 'dance_epic')
        self.assertAlmostEqual(sequence.summary()[1], 30.0)
        self.assertEqual(sequence.actions[0]['music'], {'song': epic.SONG, 'background': True})
        self.assertTrue(sequence.actions[0]['movement']['save_pose'])
        self.assertTrue(sequence.actions[-2]['movement']['restore_pose'])
        self.assertEqual(sequence.actions[-1], {'music': 'STOP'})
        self.assertEqual(sum('music' in action for action in sequence.actions), 3)
        stage = epic.make_stage()
        self.assertIn('Minimum commanded support feet: 5', stage.report())
        self.assertIn('Warnings:\n  none', stage.report())
        for name, expected in [('pickup_taps', 1.83), ('front_patter', 5.13),
                               ('six_foot_ripple', 11.62), ('driving_steps', 22.48),
                               ('fast_fill', 26.0), ('fade_steps', 29.0), ('return', 30.0)]:
            end = stage.sections[name][1]
            self.assertAlmostEqual(sum(a['movement']['duration_s'] for a in stage.actions[:end]), expected)
        fade_start, fade_end, _, _ = stage.sections['fade_steps']
        self.assertEqual(sequence.actions[fade_start]['music'], {'song': 'STOP', 'fade_out_s': 4.0})
        self.assertAlmostEqual(sum(a['movement']['duration_s'] for a in stage.actions[:fade_start]), 26.0)
        # Fast phases, many contacts across all six feet, then decreasing lift heights.
        self.assertTrue(all(0.1 <= a['movement']['duration_s'] < 0.25 for a in stage.actions[:-1]))
        lifts = [(i, leg, axes['z']) for i, a in enumerate(stage.actions)
                 for leg, axes in a['movement'].get('pose_delta', {}).get('toe_translation', {}).items()
                 if axes.get('z', 0) > 0]
        self.assertEqual(len({leg for _, leg, _ in lifts}), 6)
        self.assertGreaterEqual(len(lifts), 35)
        fading_lifts = [height for i, _, height in lifts if fade_start <= i < fade_end]
        self.assertEqual(len(fading_lifts), 4)
        self.assertTrue(all(a > b > 0 for a, b in zip(fading_lifts, fading_lifts[1:])))
        # Integrate every toe target: no planted-foot slides or unreturned offsets.
        offsets = {}
        for action in stage.actions[:-1]:
            for leg, axes in action['movement'].get('pose_delta', {}).get('toe_translation', {}).items():
                for axis, value in axes.items():
                    key = (leg, axis)
                    offsets[key] = offsets.get(key, 0) + value
        self.assertTrue(all(abs(value) < 1e-9 for value in offsets.values()))

    def test_metal_sections_contacts_and_return(self):
        self.assertEqual(metal.build().to_yaml(), (PACKAGE / 'config/sequences/dance_metal.yaml').read_text())
        self.assertEqual(metal.build().summary(), (122, 50.5))
        for bpm, amplitude in ((60, 0.5), (120, 1), (140, 1)):
            stage = metal.make_stage(bpm, amplitude)
            self.assertEqual(len(stage.sections), 8)
            self.assertTrue(all(abs(value) < 1e-9 for value in stage.state.values()))
            self.assertIn('Minimum commanded support feet: 5', stage.report())
            self.assertIn('Warnings:\n  none', stage.report())
            lifted = set()
            for action in stage.actions:
                for leg, target in action['movement'].get('pose_delta', {}).get('toe_translation', {}).items():
                    if target.get('z', 0) > 0:
                        lifted.add(leg)
            self.assertEqual(len(lifted), 6)
            for name in stage.sections:
                sequence = stage.sequence('dance_metal', name)
                sequence.summary()
                self.assertTrue(sequence.actions[0]['movement']['save_pose'])
                self.assertTrue(sequence.actions[-1]['movement']['restore_pose'])
                self.assertIn('Warnings:\n  none', stage.report(name))
        for kwargs in ({'bpm': 141}, {'bpm': 0}, {'amplitude': 1.1}, {'amplitude': float('nan')}):
            with self.assertRaises(ValueError):
                metal.build(**kwargs)

    def test_hot_dogs_grid_contacts_and_reproducibility(self):
        self.assertEqual(hot_dogs.to_yaml(),
                         (PACKAGE / 'config/sequences/snippets/dance_hot_dogs_for_breakfast.yaml').read_text())
        stage = hot_dogs.make_stage()
        self.assertEqual(len(stage.sections), 16)
        self.assertEqual(len(stage.actions), 64)
        for start, end, _, _ in stage.sections.values():
            self.assertEqual(end - start, 4)
            self.assertAlmostEqual(sum(a['movement']['duration_s'] for a in stage.actions[start:end]), 1.752)
        self.assertAlmostEqual(sum(a['movement']['duration_s'] for a in stage.actions), 28.032)
        self.assertTrue(all(abs(value) < 1e-9 for value in stage.state.values()))
        self.assertIn('Minimum commanded support feet: 5', stage.report())
        self.assertIn('Warnings:\n  none', stage.report())
        lifted = set()
        for action in stage.actions:
            for leg, offset in action['movement']['pose_delta'].get('toe_translation', {}).items():
                if offset.get('z', 0) > 0:
                    lifted.add(leg)
        self.assertEqual(len(lifted), 6)
        actions = hot_dogs.document()['actions']
        self.assertNotIn('music', actions[0])
        self.assertTrue(actions[1]['music']['background'])
        self.assertEqual(actions[-1], {'music': 'STOP'})

    def test_stage_complete_partial_and_transactional_validation(self):
        stage = Stage(120, 0.5)
        stage.cue(head={'yaw': 10, 'pitch': 6})
        stage.change(head={'yaw': 0})
        self.assertEqual(stage.state[('head_rotation', 'pitch')], 3)
        self.assertEqual(stage.state[('head_rotation', 'yaw')], 0)
        previous = dict(stage.state)
        count = len(stage.actions)
        for kwargs in ({'head': {'bad_axis': 2}}, {'feet': {'bad_leg': {'z': 1}}},
                       {'head': {'yaw': float('nan')}}, {'length': -1}):
            with self.assertRaises(ValueError):
                stage.change(**kwargs)
            self.assertEqual(stage.state, previous)
            self.assertEqual(len(stage.actions), count)
        stage.cue(body={'x': 0.01})
        self.assertNotIn(('head_rotation', 'pitch'), stage.state)

    def test_sections_and_isolated_rehearsal(self):
        stage = make_stage()
        self.assertEqual(list(stage.sections), ['intro', 'verse', 'guitar_solo', 'body_roll',
                                              'drum_fill', 'breakdown', 'encore', 'finale'])
        for name in stage.sections:
            sequence = stage.sequence('dance_rock', name)
            sequence.summary()
            self.assertEqual(sequence.name, 'dance_rock_' + name)
            self.assertTrue(sequence.actions[0]['movement']['save_pose'])
            self.assertTrue(sequence.actions[-1]['movement']['restore_pose'])
            self.assertEqual(sum(bool(a['movement'].get('save_pose')) for a in sequence.actions), 1)
            self.assertIn('Warnings:\n  none', stage.report(name))
        self.assertIn('Total: 53.43 s; 116 actions', stage.report())
        self.assertIn('Minimum commanded support feet: 5', stage.report())
        with self.assertRaises(ValueError):
            stage.sequence('dance_rock', 'missing')
        with self.assertRaises(ValueError):
            with stage.section('intro'):
                pass
        fresh = Stage(120, 1)
        for name in ('', ' '):
            with self.assertRaises(ValueError):
                with fresh.section(name):
                    pass
        with self.assertRaises(ValueError):
            with fresh.section('outer'):
                fresh.cue(head={'yaw': 10})
                with fresh.section('inner'):
                    pass
        self.assertEqual(fresh.state, {})
        self.assertEqual(fresh.actions, [])
        with fresh.section('raised'):
            fresh.cue(feet={'right_front': {'z': 0.02}})
        with self.assertRaises(ValueError):
            fresh.sequence('invalid', 'raised')

    def test_contact_phases_and_warnings(self):
        stage = Stage(120, 1)
        stage.cue(feet={'left_front': {'z': 0.02}})
        stage.change(feet={'left_front': {'x': 0.02}})
        stage.change(feet={'left_front': {'x': 0}})
        stage.cue()
        report = stage.report()
        for phase in ('lift', 'swing', 'landing', 'support'):
            self.assertIn('=' + phase, report)
        self.assertIn('Warnings:\n  none', report)
        stage.cue(feet={'right_front': {'x': 0.01, 'z': -0.01}})
        self.assertIn('below entrance ground height', stage.report())
        self.assertIn('horizontal movement during ground contact', stage.report())

    def test_rock_reproduces_catalog_and_returns_to_entrance(self):
        self.assertEqual(build_rock().to_yaml(),
                         (PACKAGE / 'config/sequences/dance_rock.yaml').read_text())
        for bpm, amplitude in ((112, 1), (60, 0.5), (140, 1)):
            sequence = build_rock(bpm, amplitude)
            self.assertEqual(sequence.summary()[0], 117)
            state = {}
            lifted = set()
            for index, action in enumerate(sequence.actions[:-1]):
                movement = action['movement']
                if movement.get('restore_pose'):
                    self.assertEqual(index, len(sequence.actions) - 2)
                    for value in state.values():
                        self.assertAlmostEqual(value, 0, places=8)
                    continue
                for field, values in movement['pose_delta'].items():
                    if field == 'toe_translation':
                        for leg, coordinates in values.items():
                            old_z = state.get((field, leg, 'z'), 0)
                            for axis, value in coordinates.items():
                                key = (field, leg, axis)
                                state[key] = state.get(key, 0) + value
                            new_z = state.get((field, leg, 'z'), 0)
                            if any(abs(coordinates.get(axis, 0)) > 1e-9 for axis in ('x', 'y')):
                                self.assertGreater(old_z, 0.001)
                                self.assertGreater(new_z, 0.001)
                            self.assertGreaterEqual(new_z, -1e-9)
                            if new_z > 0.001:
                                lifted.add(leg)
                    else:
                        for axis, value in values.items():
                            key = (field, axis)
                            state[key] = state.get(key, 0) + value
                airborne = [key for key, value in state.items()
                            if key[0] == 'toe_translation' and key[-1] == 'z' and value > 0.001]
                self.assertLessEqual(len(airborne), 1)
            self.assertEqual(len(lifted), 6)
            self.assertTrue(sequence.actions[0]['movement']['save_pose'])
            self.assertTrue(sequence.actions[-2]['movement']['restore_pose'])

    def test_rock_music_fades_through_finale_and_stops_after_restore(self):
        for bpm in (60, 112, 140):
            sequence = build_rock(bpm)
            stage = make_stage(bpm)
            finale = stage.sections['finale'][0]
            self.assertEqual(sequence.actions[0]['music'],
                             {'song': 'musicfox_excess_party.mp3', 'background': True})
            fade = sequence.actions[finale]['music']
            self.assertEqual(fade['song'], 'STOP')
            self.assertAlmostEqual(fade['fade_out_s'], 6 * 60 / bpm + 2)
            self.assertAlmostEqual(fade['fade_out_s'], sum(
                a['movement']['duration_s'] for a in sequence.actions[finale:-1]))
            self.assertEqual(sequence.actions[-1], {'music': 'STOP'})
            self.assertEqual(sum('music' in a for a in sequence.actions), 3)
            self.assertAlmostEqual(sequence.summary()[1], stage.sequence('motion').summary()[1])
            for name in stage.sections:
                self.assertTrue(all('music' not in a for a in build_rock(bpm, section=name).actions))

    def test_rock_parameter_envelope(self):
        for kwargs in ({'bpm': 0}, {'bpm': 141}, {'amplitude': 0},
                       {'amplitude': 1.1}, {'amplitude': float('nan')}):
            with self.assertRaises(ValueError):
                build_rock(**kwargs)

    def test_example_reproduces_catalog(self):
        sequence = build()
        self.assertEqual(sequence.to_yaml(), (PACKAGE / 'config/sequences/curious_look.yaml').read_text())
        count, seconds = sequence.summary()
        self.assertEqual(count, 24)
        self.assertAlmostEqual(seconds, 30.76)
        self.assertEqual(yaml.safe_load(sequence.to_yaml())['name'], 'curious_look')
        self.assertNotIn('&id', sequence.to_yaml())

    def test_composition_copies_phrases(self):
        phrase = [hold(beats(1, 120))]
        sequence = Sequence('example').extend([repeat(3, phrase)])
        phrase[0]['movement']['duration_s'] = 99
        self.assertEqual(sequence.summary(), (3, 1.5))

    def test_short_absolute_joint_save_and_restore_actions(self):
        targets = (
            {'body_pose': {'head_orientation': {'yaw': 2}}},
            {'joint_targets': {'right_front': {'torso_coxa': 0, 'coxa_femur': 50, 'femur_tibia': 90}}},
            {'save': True, 'pose_delta': {'head_rotation': {'yaw': 1}}},
            {'restore': True},
        )
        for kwargs in targets:
            for duration in (0.1, 0.5, 1.0):
                with self.subTest(kwargs=kwargs, duration=duration):
                    self.assertEqual(pose(duration, **kwargs)['movement']['duration_s'], duration)
            with self.assertRaises(ValueError):
                pose(0.099, **kwargs)
        self.assertEqual(hold(0.02)['movement']['duration_s'], 0.02)
        stage = Stage(120, 1)
        stage.cue(1, save=True, head={'yaw': 1})
        self.assertEqual(stage.actions[0]['movement']['duration_s'], 0.5)

    def test_invalid_inputs(self):
        for duration in (0, -1, float('nan'), float('inf'), True):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                hold(duration)
        for kwargs in (
            {'body_pose': {'head_orientation': {}}, 'duration_s': 0.099},
            {'pose_delta': {'toe_translation': {'bad_leg': {'z': 1}}}},
            {'pose_delta': {'head_rotation': {'yaw': float('nan')}}},
            {'body_pose': {'toe_positions': {'left_front': {'x': 0, 'y': 0, 'z': 0}}}},
            {'pose_delta': {}, 'body_pose': {'head_orientation': {}}},
            {'restore': True, 'save': True},
            {'interpolation': 'unknown', 'pose_delta': {'head_rotation': {}}},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                pose(**({'duration_s': 1} | kwargs))
        for count in (0, 1001, True, 1.5):
            with self.assertRaises(ValueError):
                repeat(count, [hold(1)])
        with self.assertRaises(ValueError):
            beats(1, 0)

    def test_save_restore_flow_and_limits(self):
        saving = pose(1, save=True, pose_delta={'head_rotation': {'yaw': 5}})
        restoring = pose(1, restore=True)
        for actions in ([restoring], [saving], [saving, saving, restoring],
                        [repeat(2, [saving]), restoring],
                        [repeat(1000, [repeat(11, [hold(1)])])]):
            with self.assertRaises(ValueError):
                Sequence('invalid').extend(actions).to_yaml()
        self.assertEqual(Sequence('valid').extend([repeat(2, [saving, restoring])]).summary(), (4, 4))
        nested = hold(1)
        for _ in range(17):
            nested = repeat(1, [nested])
        with self.assertRaises(ValueError):
            Sequence('deep').extend([nested]).summary()

    def test_export_protects_existing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'sequence.yaml'
            sequence = build()
            sequence.write(path)
            with self.assertRaises(FileExistsError):
                sequence.write(path)
            self.assertEqual(path.read_text(), sequence.to_yaml())
            sequence.write(path, force=True)

    def test_targets_and_triggers(self):
        action = pose(1, body_pose={'head_orientation': {}}, joint_targets={
            'right_front': {'torso_coxa': -20, 'coxa_femur': 50, 'femur_tibia': 60}})
        sequence = Sequence('joint_example', trigger={'voice': 'example'}).extend([action])
        self.assertEqual(yaml.safe_load(sequence.to_yaml())['trigger'], {'voice': 'example'})


if __name__ == '__main__':
    unittest.main()
