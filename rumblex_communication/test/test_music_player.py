from unittest.mock import Mock, patch
import subprocess
import sys

from rumblex_communication.music_player import MusicPlayer


def test_import_does_not_validate_legacy_entrypoint_dependencies():
    # Reproduce a setuptools console script in a fresh interpreter. An unrelated
    # unsatisfied requirement must not be evaluated just by importing audio.
    result = subprocess.run(
        [sys.executable, '-c',
         "__requires__ = 'rumblex-intentionally-missing-test-dependency'; "
         'from rumblex_communication.music_player import MusicPlayer; '
         "import sys; assert 'pkg_resources' not in sys.modules"],
        capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 0, result.stderr


def test_completion_waits_for_playback_and_runs_once():
    with patch('rumblex_communication.music_player.mixer') as mixer:
        player = MusicPlayer()
        done = Mock()
        player.play_file('speech.mp3', cb=done)
        mixer.music.get_busy.return_value = True
        player.poll()
        done.assert_not_called()
        mixer.music.get_busy.return_value = False
        player.poll()
        player.poll()
        done.assert_called_once_with()


def test_stop_discards_previous_completion():
    with patch('rumblex_communication.music_player.mixer') as mixer:
        player = MusicPlayer()
        done = Mock()
        player.play_file('speech.mp3', cb=done)
        player.stop_playing()
        mixer.music.get_busy.return_value = False
        player.poll()
        done.assert_not_called()
