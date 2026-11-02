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
        player.play('speech.mp3', cb=done)
        mixer.music.play.assert_called_once_with()
        mixer.Sound.assert_not_called()
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


def test_background_music_survives_speech_and_foreground_cancellation():
    with patch('rumblex_communication.music_player.mixer') as mixer:
        player = MusicPlayer()
        player.play('party.mp3', background=True)
        sound = mixer.Sound.return_value
        channel = sound.play.return_value
        player.play_file('speech.mp3')
        player.stop_playing()
        channel.stop.assert_not_called()
        player.stop_background()
        channel.stop.assert_called_once_with()


def test_new_background_track_replaces_previous_and_close_stops_it():
    with patch('rumblex_communication.music_player.mixer') as mixer:
        first, second = Mock(), Mock()
        mixer.Sound.return_value.play.side_effect = [first, second]
        player = MusicPlayer()
        player.play('first.mp3', background=True)
        player.play('second.mp3', background=True)
        first.stop.assert_called_once_with()
        second.stop.assert_not_called()
        player.close()
        second.stop.assert_called_once_with()


def test_background_music_rejects_invalid_paths():
    import pytest

    with patch('rumblex_communication.music_player.mixer') as mixer:
        player = MusicPlayer()
        for filename in ('../party.mp3', 'party.txt'):
            with pytest.raises(ValueError):
                player.play(filename, background=True)
        mixer.Sound.assert_not_called()
