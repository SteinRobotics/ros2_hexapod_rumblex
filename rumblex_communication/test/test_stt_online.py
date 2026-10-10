from unittest.mock import Mock, patch

import speech_recognition as sr
from speech_recognition.recognizers.google_cloud import _build_config

from rumblex_communication.stt_online import SpeechToTextOnline


def test_online_recognition_builds_valid_german_cloud_config():
    callback = Mock()
    audio = sr.AudioData(b'\x00\x00' * 160, 16000, 2)

    def recognize(audio_data, **kwargs):
        # Exercise the installed library's config builder to catch invalid fields.
        config = _build_config(audio_data, kwargs)
        assert config.language_code == 'de-DE'
        return 'gehe vorwärts'

    with patch('rumblex_communication.stt_online.sr.Recognizer') as recognizer, \
            patch('rumblex_communication.stt_online.sr.Microphone'):
        recognizer.return_value.listen.return_value = audio
        recognizer.return_value.recognize_google_cloud.side_effect = recognize
        SpeechToTextOnline(callback, logger=Mock()).run()

    callback.assert_called_once_with('gehe vorwärts')


def test_cloud_request_failure_completes_with_fallback():
    callback = Mock()
    with patch('rumblex_communication.stt_online.sr.Recognizer') as recognizer, \
            patch('rumblex_communication.stt_online.sr.Microphone'):
        recognizer.return_value.recognize_google_cloud.side_effect = sr.RequestError('offline')
        SpeechToTextOnline(callback, logger=Mock()).run()

    callback.assert_called_once_with('das habe ich nicht verstanden')
