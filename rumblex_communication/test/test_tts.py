from pathlib import Path
from unittest.mock import Mock, patch

from rumblex_communication.tts import TextToSpeech


def make_tts(tmp_path):
    tts = TextToSpeech(Mock(), 'de', logger=Mock())
    tts.tts_cache = tmp_path / 'missing' / 'tts_cache'
    return tts


def test_first_request_creates_cache_and_uses_google(tmp_path):
    tts = make_tts(tmp_path)
    callback = Mock()
    with patch('rumblex_communication.tts.gTTS') as google, \
            patch('rumblex_communication.tts.subprocess.call') as fallback:
        google.return_value.save.side_effect = lambda path: Path(path).write_bytes(b'mp3 audio')
        tts.run('Hallo!', callback)

    google.assert_called_once_with(text='Hallo!', lang='de')
    cached = tts.tts_cache / 'hallo.mp3'
    assert cached.read_bytes() == b'mp3 audio'
    tts.music_player.play_file.assert_called_once_with(str(cached))
    fallback.assert_not_called()
    callback.assert_called_once_with()


def test_cached_audio_does_not_contact_google(tmp_path):
    tts = make_tts(tmp_path)
    tts.tts_cache.mkdir(parents=True)
    cached = tts.tts_cache / 'hallo.mp3'
    cached.write_bytes(b'mp3 audio')
    with patch('rumblex_communication.tts.gTTS') as google, \
            patch('rumblex_communication.tts.subprocess.call') as fallback:
        tts.run('Hallo!')

    google.assert_not_called()
    fallback.assert_not_called()
    tts.music_player.play_file.assert_called_once_with(str(cached))


def test_failed_download_is_not_cached_and_next_request_retries(tmp_path):
    tts = make_tts(tmp_path)
    callback = Mock()

    def fail_download(path):
        Path(path).write_bytes(b'partial audio')
        raise RuntimeError('Request failed')

    with patch('rumblex_communication.tts.gTTS') as google, \
            patch('rumblex_communication.tts.subprocess.call') as fallback:
        google.return_value.save.side_effect = fail_download
        tts.run('Hallo!', callback)
        assert list(tts.tts_cache.iterdir()) == []
        fallback.assert_called_once()
        tts.music_player.play_file.assert_not_called()
        callback.assert_called_once_with()

        google.return_value.save.side_effect = lambda path: Path(path).write_bytes(b'mp3 audio')
        tts.run('Hallo!')

    assert google.call_count == 2
    assert (tts.tts_cache / 'hallo.mp3').read_bytes() == b'mp3 audio'
    tts.music_player.play_file.assert_called_once()
    assert fallback.call_count == 1


def test_empty_cache_entry_is_downloaded_again(tmp_path):
    tts = make_tts(tmp_path)
    tts.tts_cache.mkdir(parents=True)
    cached = tts.tts_cache / 'hallo.mp3'
    cached.touch()
    with patch('rumblex_communication.tts.gTTS') as google, \
            patch('rumblex_communication.tts.subprocess.call') as fallback:
        google.return_value.save.side_effect = lambda path: Path(path).write_bytes(b'mp3 audio')
        tts.run('Hallo!')

    assert cached.read_bytes() == b'mp3 audio'
    google.return_value.save.assert_called_once()
    fallback.assert_not_called()
