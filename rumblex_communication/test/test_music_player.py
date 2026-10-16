from unittest.mock import Mock, patch

from rumblex_communication.music_player import MusicPlayer


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
