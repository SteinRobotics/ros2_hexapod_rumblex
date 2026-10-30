import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rumblex_teleop.sequence_catalog import load_catalog


def test_repository_catalog():
    directory = Path(__file__).resolve().parents[2] / 'rumblex_brain/config/sequences'
    catalog = load_catalog(directory)
    assert catalog.extra_sequences == (
        'calibrate', 'clap', 'dance_dancing_street', 'dance_standard', 'hold_distance',
        'level_torso', 'neutral', 'run_demo', 'waiting', 'walk_demo')
    assert len(catalog.captions) == 12
    assert catalog.captions['a'] == 'normal_watch'
    assert catalog.captions['dpad_up'] == 'standup'
    assert catalog.captions['r2'] == 'dance_hot_dogs_for_breakfast'


def test_discovery_wrappers_and_new_sequences(tmp_path):
    (tmp_path / 'snippets').mkdir()
    (tmp_path / 'snippets/ignored.yaml').write_text('actions: []')
    (tmp_path / 'b.yml').write_text('behaviors:\n  - name: second\n')
    (tmp_path / 'a.yaml').write_text(
        'sequences:\n  - name: mapped\n    trigger: {joystick: button_x}\n  - name: first\n')
    catalog = load_catalog(tmp_path)
    assert catalog.captions == {'x': 'mapped'}
    assert catalog.extra_sequences == ('first', 'second')
    (tmp_path / 'c.yaml').write_text('name: future_sequence\n')
    assert load_catalog(tmp_path).extra_sequences == ('first', 'second', 'future_sequence')


def test_reserved_and_shadowed_mappings_remain_accessible(tmp_path):
    (tmp_path / 'a.yaml').write_text('sequences:\n'
                                  '  - name: first\n    trigger: {joystick: button_a}\n'
                                  '  - name: shadowed\n    trigger: {joystick: button_a}\n'
                                  '  - name: reserved\n    trigger: {joystick: button_start}\n')
    catalog = load_catalog(tmp_path)
    assert catalog.captions == {'a': 'first'}
    assert catalog.extra_sequences == ('shadowed', 'reserved')


@pytest.mark.parametrize('contents', [
    '[]', 'name: ""', 'sequences: {}', 'sequences: [null]',
    'sequences: [{name: duplicate}, {name: duplicate}]',
    'name: invalid\ntrigger: []',
    'sequences: []\nbehaviors: []',
    'name: invalid\ntrigger: {joystick: []}',
])
def test_invalid_catalog_fails_as_a_whole(tmp_path, contents):
    (tmp_path / 'sequence.yaml').write_text(contents)
    with pytest.raises(ValueError):
        load_catalog(tmp_path)


def test_missing_directory(tmp_path):
    with pytest.raises(ValueError, match='does not exist'):
        load_catalog(tmp_path / 'missing')
