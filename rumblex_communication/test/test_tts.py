import json
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from rumblex_communication.tts import TextToSpeech


def make_tts(tmp_path):
    tts = TextToSpeech('de', logger=Mock())
    tts.tts_cache = tmp_path / 'cache'
    return tts


def test_download_is_cached(tmp_path):
    tts = make_tts(tmp_path)
    with patch('rumblex_communication.tts.gTTS') as google:
        google.return_value.save.side_effect = lambda path: Path(path).write_bytes(b'audio')
        path = tts.synthesize('Hallo!')
        assert tts.synthesize('Hallo!') == path
    assert path.read_bytes() == b'audio'
    google.assert_called_once_with(text='Hallo!', lang='de', timeout=10)
    assert json.loads(path.with_suffix('.mp3.json').read_text()) == {
        'text': 'Hallo!', 'language': 'de', 'engine': 'gtts', 'audio_file': path.name,
    }


def test_cache_distinguishes_punctuation_case_long_text_and_language(tmp_path):
    tts = make_tts(tmp_path)
    with patch('rumblex_communication.tts.gTTS') as google:
        google.return_value.save.side_effect = lambda path: Path(path).write_bytes(b'audio')
        texts = ('Hallo!', 'hallo', 'Grüße! ' * 1000 + 'b', 'Grüße! ' * 1000 + 'c')
        paths = [tts.synthesize(text) for text in texts]
        for path, text in zip(paths, texts):
            assert len(path.name) == 68
            assert json.loads(path.with_suffix('.mp3.json').read_text())['text'] == text
        tts.language = 'en'
        paths.append(tts.synthesize('Hallo!'))
    assert len(set(paths)) == 5


def test_failed_download_uses_wav_and_retries_online_next_time(tmp_path):
    tts = make_tts(tmp_path)
    with patch('rumblex_communication.tts.gTTS') as google, \
            patch('rumblex_communication.tts.subprocess.run') as fallback:
        def fail(path):
            Path(path).write_bytes(b'partial')
            raise RuntimeError('offline')
        google.return_value.save.side_effect = fail
        fallback.side_effect = lambda args, **kwargs: Path(args[4]).write_bytes(b'wav')
        path = tts.synthesize('Hallo!')
        assert path.suffix == '.wav'
        metadata = json.loads(path.with_suffix('.wav.json').read_text())
        assert metadata == {
            'text': 'Hallo!', 'language': 'de', 'engine': 'espeak-ng',
            'audio_file': path.name, 'voice': 'de+f3',
        }
        assert not list(tts.tts_cache.glob('*.mp3'))
        google.return_value.save.side_effect = lambda path: Path(path).write_bytes(b'mp3')
        assert tts.synthesize('Hallo!').suffix == '.mp3'
        assert json.loads(path.with_suffix('.wav.json').read_text()) == metadata
    assert google.call_count == 2


def test_fallback_failure_propagates_and_cleans_temporary_files(tmp_path):
    tts = make_tts(tmp_path)
    with patch('rumblex_communication.tts.gTTS', side_effect=RuntimeError('offline')), \
            patch('rumblex_communication.tts.subprocess.run', side_effect=FileNotFoundError):
        with pytest.raises(FileNotFoundError):
            tts.synthesize('Hallo!')
    assert list(tts.tts_cache.iterdir()) == []


def test_cache_is_package_local_regardless_of_cwd_and_xdg(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    monkeypatch.chdir(tmp_path)
    assert TextToSpeech('de').tts_cache == Path(__file__).resolve().parents[1] / 'tts_cache'
