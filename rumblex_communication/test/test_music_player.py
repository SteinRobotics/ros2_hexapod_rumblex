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


def test_background_fade_is_nonblocking_and_preserves_speech():
    with patch('rumblex_communication.music_player.mixer') as mixer:
        player = MusicPlayer()
        player.play('party.mp3', background=True)
        channel = mixer.Sound.return_value.play.return_value
        done = Mock()
        player.play_file('speech.mp3', cb=done)
        mixer.music.reset_mock()
        player.fade_background(4)
        channel.fadeout.assert_called_once_with(4000)
        channel.stop.assert_not_called()
        mixer.music.stop.assert_not_called()
        mixer.music.fadeout.assert_not_called()
        mixer.music.get_busy.return_value = False
        player.poll()
        done.assert_called_once_with()
        # Replacing a fading track cancels it immediately, not the new track.
        replacement = Mock()
        mixer.Sound.return_value.play.return_value = replacement
        player.play('next.mp3', background=True)
        channel.stop.assert_called_once_with()
        replacement.stop.assert_not_called()
        replacement.fadeout.assert_not_called()


def test_background_fade_validates_duration_and_handles_no_track_or_zero():
    import pytest

    with patch('rumblex_communication.music_player.mixer') as mixer:
        player = MusicPlayer()
        player.fade_background(4)
        mixer.Sound.assert_not_called()
        player.play('party.mp3', background=True)
        channel = mixer.Sound.return_value.play.return_value
        for duration in (-1, float('nan'), float('inf'), 61):
            with pytest.raises(ValueError):
                player.fade_background(duration)
        channel.fadeout.assert_not_called()
        channel.stop.assert_not_called()
        player.fade_background(0)
        channel.stop.assert_called_once_with()


def test_real_background_channel_fades_to_silence_without_blocking():
    # Exercise SDL's real fade behavior without opening speakers or audio files.
    result = subprocess.run(
        [sys.executable, '-c', '''
import os
import time
os.environ['SDL_AUDIODRIVER'] = 'dummy'
from rumblex_communication.music_player import MusicPlayer, mixer
player = MusicPlayer()
try:
    frequency, format_, channels = mixer.get_init()
    sound = mixer.Sound(buffer=bytes(frequency * channels * (abs(format_) // 8)))
    player._background_channel = sound.play(loops=-1)
    channel = player._background_channel
    player.fade_background(0.3)
    assert channel.get_busy(), 'fade blocked until playback stopped'
    deadline = time.monotonic() + 2
    while channel.get_busy() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not channel.get_busy(), 'fade did not stop playback'
finally:
    player.close()
'''], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
