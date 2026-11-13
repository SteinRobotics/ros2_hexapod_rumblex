"""Offline sequence generation and editing tests; GUI cases require a display."""
import copy
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
from sequence_generator import (
    GAITS, PRESETS, Sequence, arc, builtin, hold, integrate, load_yaml, move, pose,
    preset, repeat, route_preview, sequence_notes, stop, travel, turn,
)


class MovementAuthoringTest(unittest.TestCase):
    def test_travel_turn_and_arc_units(self):
        action = travel(-0.3, 0.03, bearing_deg=90)
        self.assertAlmostEqual(action['movement']['duration_s'], 10)
        self.assertAlmostEqual(action['velocity']['linear']['y'], -0.03)
        self.assertAlmostEqual(action['velocity']['linear']['x'], 0)
        action = turn(-90, 0.2)
        self.assertAlmostEqual(action['movement']['duration_s'], math.pi / 0.4)
        self.assertEqual(action['velocity']['angular']['z'], -0.2)
        action = arc(0.5, -180, 0.1)
        self.assertAlmostEqual(action['movement']['duration_s'], math.pi / 0.2)
        self.assertEqual(action['velocity']['linear']['x'], 0.1)
        self.assertEqual(action['velocity']['angular']['z'], -0.2)

    def test_every_gait_and_explicit_velocity_reset(self):
        for gait in GAITS:
            action = move(2, x=0.03, gait=gait)
            self.assertEqual(action['movement']['name'], gait)
            self.assertEqual(action['velocity']['angular'], dict(x=0.0, y=0.0, z=0.0))
            self.assertEqual(action['velocity']['linear'], dict(x=0.03, y=0.0, z=0.0))
            sequence = Sequence('route').extend([action, stop()])
            self.assertEqual(sequence.summary(), (2, 2.1))
            self.assertFalse(any('Add Stop' in note for note in sequence_notes(sequence)))

    def test_invalid_motion_inputs(self):
        cases = [lambda: move(0), lambda: move(True), lambda: move(1, yaw=math.nan),
                 lambda: move(1, x=math.inf), lambda: move(1, gait='SINGLE_POSE'),
                 lambda: travel(0), lambda: travel(1, 0), lambda: travel(1, bearing_deg=math.inf),
                 lambda: turn(0), lambda: turn(90, -1), lambda: arc(0), lambda: arc(1, 0),
                 lambda: builtin('HOLD_DISTANCE'), lambda: builtin('LEVEL_TORSO', distance_m=0.5)]
        for call in cases:
            with self.subTest(call=call), self.assertRaises(ValueError):
                call()

    def test_presets_roundtrip_and_closed_routes(self):
        for name in PRESETS:
            sequence = preset(name)
            restored = Sequence.from_yaml(sequence.to_yaml())
            self.assertEqual(restored.actions, sequence.actions)
            self.assertEqual(restored.summary(), sequence.summary())
        for clockwise in (False, True):
            for name in ('square', 'circle', 'figure_eight'):
                with self.subTest(name=name, clockwise=clockwise):
                    sequence = preset(name, clockwise=clockwise)
                    points = route_preview(sequence)
                    self.assertAlmostEqual(points[-1].x, 0, places=9)
                    self.assertAlmostEqual(points[-1].y, 0, places=9)
                    angle = 0 if name == 'figure_eight' else (-2 if clockwise else 2) * math.pi
                    self.assertAlmostEqual(points[-1].yaw, angle)
                    self.assertEqual(sequence.actions[-1], stop())
        square = preset('square')
        self.assertEqual(square.summary()[0], 10)
        self.assertAlmostEqual(square.summary()[1], 1.5 + 4 * (0.5 / 0.03 + math.pi / 0.4) + 0.1)

    def test_route_body_frame_and_persistent_velocity(self):
        sequence = Sequence('body_frame').extend([turn(90), travel(1, 0.1),
                                                 {'movement': {'name': GAITS[0], 'duration_s': 1}}, stop()])
        endpoint = route_preview(sequence)[-1]
        self.assertAlmostEqual(endpoint.x, 0)
        self.assertAlmostEqual(endpoint.y, 1.1)
        # A single-pose hold is not a walking gait, even while velocity remains set.
        sequence = Sequence('hold').extend([travel(1, 0.1), hold(10)])
        self.assertAlmostEqual(route_preview(sequence)[-1].x, 1)
        self.assertTrue(any('Add Stop' in note for note in sequence_notes(sequence)))

    def test_preview_sampling_is_bounded_and_analytic(self):
        points = route_preview(Sequence('long').extend([move(1e6, yaw=0.01), stop()]))
        self.assertLess(len(points), 30)
        self.assertAlmostEqual(points[-1].yaw, 10000)
        self.assertEqual(integrate(0, 0, 0, 1, 0, 0, 2), (2, 0, 0))
        for rate in (0, 1e-15):
            x, y, heading = integrate(0, 0, math.pi / 2, 1, 0, rate, 2)
            self.assertAlmostEqual(x, 0)
            self.assertAlmostEqual(y, 2)

    def test_mixed_audio_sensor_and_pose_actions(self):
        actions = [builtin('HOLD_DISTANCE', 4, distance_m=0.6), builtin('LEVEL_TORSO', 3),
                   pose(0.5, save=True, pose_delta={'head_rotation': {'yaw': 5}}),
                   {'say': {'text': 'Hi', 'minDuration': 2}, 'music': {'song': 'song.mp3', 'background': True}},
                   pose(0.5, restore=True)]
        sequence = Sequence('mixed').extend(actions)
        self.assertEqual(sequence.summary(), (5, 10))
        self.assertTrue(any('Sensor-driven' in note for note in sequence_notes(sequence)))
        self.assertTrue(any('Audio' in note for note in sequence_notes(sequence)))
        invalid = builtin('HOLD_DISTANCE', 4, distance_m=0.5)
        invalid['velocity'] = {'linear': {'x': 0.1}}
        with self.assertRaisesRegex(ValueError, 'own their velocity'):
            Sequence('invalid').extend([invalid]).summary()

    def test_background_music_fade_round_trip_and_validation(self):
        sequence = Sequence('fade').extend([{'music': {'song': 'STOP', 'fade_out_s': 4}}])
        self.assertEqual(Sequence.from_yaml(sequence.to_yaml()).actions, sequence.actions)
        self.assertEqual(sequence.summary(), (1, 0))
        for song, fade in [('STOP', -1), ('STOP', math.nan), ('STOP', math.inf),
                           ('STOP', 61), ('STOP', True), ('track.mp3', 4)]:
            with self.subTest(song=song, fade=fade), self.assertRaises(ValueError):
                Sequence('bad').extend([{'music': {'song': song, 'fade_out_s': fade}}]).summary()

    def test_import_errors_are_actionable_and_no_lossy_unknown_fields(self):
        documents = [[], {'name': 'x', 'actions': []}, {'name': 'x', 'actions': [None]},
                     {'name': 'x', 'actions': [{'include': 'missing.yaml'}]},
                     {'name': 'x', 'imports': [], 'actions': [hold(1)]},
                     {'name': 'x', 'actions': [{'movement': None}]},
                     {'name': 'x', 'actions': [{'movement': {'name': 'UNKNOWN'}}]},
                     {'name': 'x', 'actions': [{'say': {'text': 3}}]},
                     {'name': 'x', 'actions': [{'velocity': {'linear': {'x': math.nan}}}]},
                     {'name': 'x', 'actions': [{'music': {'volume': 2}}]},
                     {'name': 'x', 'actions': [{'movement': {'name': 'SINGLE_POSE', 'duration_s': 1}}]}]
        for document in documents:
            with self.subTest(document=document), self.assertRaises(ValueError):
                Sequence.from_yaml(yaml.safe_dump(document))
        for content in ('a: 1\na: 2', 'x: &x [*x]', 'a: [', '[1, 2]: 3', 'say: ${kGreeting}'):
            with self.subTest(content=content), self.assertRaises(ValueError):
                load_yaml(content)

    def test_anchor_import_and_mutable_metadata(self):
        sequence = Sequence.from_yaml('''name: anchored
variables:
  kDuration: &duration 2.0
actions:
  - movement: {name: MOVE_TRIPOD, duration_s: *duration}
    velocity: {linear: {x: 0.03}, angular: {z: 0}}
  - movement: {name: CONTINUOUS_POSE}
    velocity: {linear: {x: 0}, angular: {z: 0}}
''')
        self.assertEqual(sequence.summary(), (2, 3.0))
        sequence.name = ''
        with self.assertRaises(ValueError):
            sequence.to_yaml()

    def test_cli_stdout_export_overwrite_and_validation(self):
        command = [sys.executable, str(TOOLS / 'sequence_generator.py'), '--preset', 'square']
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        self.assertEqual(Sequence.from_yaml(result.stdout).summary()[0], 10)
        self.assertIn('10 expanded actions', result.stderr)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.yaml'
            subprocess.run([*command, '--output', str(path)], capture_output=True, check=True)
            before = path.read_bytes()
            failed = subprocess.run([*command, '--output', str(path)], capture_output=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertEqual(path.read_bytes(), before)
            subprocess.run([*command, '--output', str(path), '--force'], capture_output=True, check=True)
            invalid = subprocess.run([*command, '--speed', 'nan'], capture_output=True)
            self.assertNotEqual(invalid.returncode, 0)

    def test_failed_atomic_export_preserves_existing_file(self):
        sequence = preset('forward')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.yaml'
            path.write_text('original\n')
            with patch('sequence_generator.os.replace', side_effect=OSError('write failed')):
                with self.assertRaises(OSError):
                    sequence.write(path, force=True)
            self.assertEqual(path.read_text(), 'original\n')
            self.assertEqual(list(Path(directory).iterdir()), [path])
            sequence.actions.append(pose(1, restore=True))
            with self.assertRaises(ValueError):
                sequence.write(path, force=True)
            self.assertEqual(path.read_text(), 'original\n')

    def test_bounded_yaml_and_expanded_motion(self):
        for content in ('x: ' + 'a' * 2_000_000, 'x: ' + '[' * 70 + '0' + ']' * 70):
            with self.assertRaises(ValueError):
                load_yaml(content)
        with self.assertRaisesRegex(ValueError, '10000'):
            Sequence('too_long').extend([repeat(1000, [repeat(11, [move(1)])])]).summary()


@unittest.skipUnless(os.environ.get('RUMBLEX_TEST_GUI') == '1', 'Set RUMBLEX_TEST_GUI=1 in a desktop session')
class EditorTest(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        from sequence_generator_gui import SequenceEditor
        self.root = tk.Tk()
        self.root.withdraw()
        self.editor = SequenceEditor(self.root, preset('square'))
        self.root.update()

    def tearDown(self):
        self.editor.pause()
        self.root.destroy()

    def select(self, index):
        self.editor.tree.selection_set(str(index))
        self.root.update()

    def test_add_edit_undo_redo_delete(self):
        e = self.editor
        self.select(0)
        e.kind.set('Travel')
        e.compose_fields()
        e.fields['distance'].set('0.1')
        e.add_block()
        self.assertEqual(len(e.sequence.actions), 4)
        self.assertAlmostEqual(e.sequence.actions[1]['movement']['duration_s'], 0.1 / 0.03)
        e.undo()
        self.assertEqual(len(e.sequence.actions), 3)
        e.redo()
        self.assertEqual(len(e.sequence.actions), 4)
        self.select(1)
        e.block_text.delete('1.0', 'end')
        e.block_text.insert('1.0', yaml.safe_dump(turn(45)))
        self.assertTrue(e.apply_block())
        self.assertEqual(e.sequence.actions[1], turn(45))
        e.delete()
        self.assertEqual(len(e.sequence.actions), 3)

    def test_forms_invalid_edits_and_save_pair_drafts(self):
        e = self.editor
        for kind in ('Travel', 'Turn', 'Arc', 'Velocity', 'Pose offset', 'Hold', 'Save pose',
                     'Restore pose', 'Stand up', 'Lay down', 'Stop', 'Say', 'Music', 'Hold distance', 'Level torso'):
            e.kind.set(kind)
            e.compose_fields()
            from sequence_generator import validate_action
            validate_action(e.make_block())
        e.replace_document(Sequence('draft'))
        e.kind.set('Save pose')
        e.compose_fields()
        e.add_block()
        self.assertIn('restored', e.status.get())
        e.kind.set('Restore pose')
        e.compose_fields()
        e.add_block()
        self.assertEqual(e.sequence.summary()[0], 2)
        self.select(0)
        before = copy.deepcopy(e.sequence.actions)
        e.block_text.delete('1.0', 'end')
        e.block_text.insert('1.0', 'movement: {name: INVALID}')
        with patch('sequence_generator_gui.messagebox.showerror') as error:
            self.assertFalse(e.apply_block())
            error.assert_called_once()
        self.assertEqual(e.sequence.actions, before)

    def test_loop_reorder_preview_and_export(self):
        e = self.editor
        self.select(0)
        e.duplicate()
        self.root.update()
        with patch('sequence_generator_gui.simpledialog.askinteger', return_value=2):
            e.tree.selection_set('0', '1')
            self.root.update()
            e.wrap_loop()
        self.assertEqual(e.sequence.actions[0]['loop']['times'], 2)
        e.shift(1)
        self.assertIn('loop', e.sequence.actions[1])
        self.root.deiconify()
        self.root.update()
        e.draw_route()
        e.scrub.set(e.points[-1].seconds / 2)
        e.draw_cursor()
        self.assertTrue(e.canvas.find_withtag('cursor'))
        e.toggle_play()
        self.assertTrue(e.playing)
        e.pause()
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'gui.yaml')
            with patch('sequence_generator_gui.filedialog.asksaveasfilename', return_value=path):
                self.assertTrue(e.export())
            self.assertEqual(Sequence.from_yaml(Path(path).read_text()).actions, e.sequence.actions)

    def test_discard_cancel_and_open_errors_preserve_document(self):
        e = self.editor
        self.select(0)
        e.block_text.insert('end', '\n# draft edit')
        with patch('sequence_generator_gui.messagebox.askyesnocancel', return_value=None):
            self.assertFalse(e.settle_block())
        with patch('sequence_generator_gui.messagebox.askyesnocancel', return_value=False):
            self.assertTrue(e.settle_block())
        self.assertFalse(e.block_text.edit_modified())
        with patch('sequence_generator_gui.filedialog.askopenfilename', return_value='/tmp/no-such-sequence.yaml'), \
                patch('sequence_generator_gui.messagebox.showerror') as error:
            e.open_file()
            error.assert_called_once()
        self.assertEqual(e.sequence.name, 'square')

    def test_small_window_controls_and_pending_selection(self):
        e = self.editor
        self.root.deiconify()
        self.root.geometry('1080x720')
        e.kind.set('Pose offset')
        e.compose_fields()
        self.root.update()

        def descendants(widget):
            for child in widget.winfo_children():
                yield child
                yield from descendants(child)

        labels = {'+ Insert after selection', 'Apply block edits', 'Export YAML…', '▶ Play'}
        found = set()
        for widget in descendants(self.root):
            if widget.winfo_class() == 'TButton' and widget.cget('text') in labels:
                # The preset tab is hidden; only test the visible authoring controls.
                found.add(widget.cget('text'))
                self.assertTrue(widget.winfo_ismapped())
                bottom = widget.winfo_rooty() - self.root.winfo_rooty() + widget.winfo_height()
                self.assertLessEqual(bottom, self.root.winfo_height())
        self.assertEqual(found, labels)
        self.select(0)
        e.block_text.insert('end', '\n# retain this edit')
        with patch('sequence_generator_gui.messagebox.askyesnocancel', return_value=True):
            self.select(1)
        self.assertEqual(e.selected, 1)
        self.assertEqual(e.tree.selection(), ('1',))


if __name__ == '__main__':
    unittest.main()
