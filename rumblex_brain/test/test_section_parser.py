"""Generate isolated sections in a temporary directory and parse with the C++ runtime."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from examples.dance_rock import make_stage

with tempfile.TemporaryDirectory(prefix='rumblex-rehearsal-') as directory:
    stage = make_stage()
    for name in stage.sections:
        stage.sequence('dance_rock', name).write(Path(directory) / f'{name}.yaml')
    result = subprocess.run(
        [sys.argv[1], '--gtest_filter=SequenceParserTest.GeneratedSectionRehearsals'],
        env=dict(os.environ, RUMBLEX_REHEARSAL_DIRECTORY=directory), check=False)
    sys.exit(result.returncode)
