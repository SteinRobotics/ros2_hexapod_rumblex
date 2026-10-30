"""Sequence names and controller captions from the brain's installed catalog."""

from dataclasses import dataclass
from pathlib import Path

import yaml


TRIGGER_CONTROLS = {
    # Start is intercepted by the coordinator for gait cycling.
    **{f'button_{name}': name for name in
       ('a', 'b', 'x', 'y', 'l1', 'l2', 'r1', 'r2', 'select', 'home')},
    'dpad_vertical_up': 'dpad_up', 'dpad_vertical_down': 'dpad_down',
    'dpad_horizontal_left': 'dpad_left', 'dpad_horizontal_right': 'dpad_right',
}


@dataclass(frozen=True)
class SequenceCatalog:
    captions: dict
    extra_sequences: tuple


def load_catalog(directory):
    """Read metadata only; the brain remains responsible for action validation."""
    directory = Path(directory)
    if not directory.is_dir():
        raise ValueError(f'Sequence directory does not exist: {directory}')
    captions = {}
    extras = []
    names = set()
    for path in sorted(directory.iterdir()):
        if not path.is_file() or path.suffix not in ('.yaml', '.yml'):
            continue
        document = yaml.safe_load(path.read_text())
        if not isinstance(document, dict):
            raise ValueError(f'Expected a sequence mapping in {path.name}')
        if 'sequences' in document and 'behaviors' in document:
            raise ValueError(f'Cannot mix sequences and behaviors wrappers in {path.name}')
        sequences = document.get('sequences', document.get('behaviors', [document]))
        if not isinstance(sequences, list):
            raise ValueError(f'Expected a sequence list in {path.name}')
        for sequence in sequences:
            if not isinstance(sequence, dict):
                raise ValueError(f'Expected a named sequence in {path.name}')
            name = sequence.get('name')
            if not isinstance(name, str) or not name or name in names:
                raise ValueError(f'Missing or duplicate sequence name in {path.name}')
            names.add(name)
            trigger = sequence.get('trigger', {})
            if not isinstance(trigger, dict):
                raise ValueError(f'Expected trigger mapping for {name}')
            joystick = trigger.get('joystick')
            if joystick is not None and not isinstance(joystick, str):
                raise ValueError(f'Expected joystick trigger string for {name}')
            control = TRIGGER_CONTROLS.get(joystick)
            if control is None:
                extras.append(name)
            else:
                # The brain uses the first matching trigger in filename order.
                captions.setdefault(control, name)
                if captions[control] != name:
                    extras.append(name)
    return SequenceCatalog(captions, tuple(extras))


def installed_catalog():
    from ament_index_python.packages import get_package_share_directory

    return load_catalog(Path(get_package_share_directory('rumblex_brain')) / 'config' / 'sequences')
