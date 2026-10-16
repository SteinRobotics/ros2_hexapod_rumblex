"""Speech synthesis with a writable, content-addressed audio cache."""

import hashlib
import logging
import os
from pathlib import Path
import subprocess
import tempfile

from gtts import gTTS


class TextToSpeech:
    def __init__(self, language, logger=None):
        self.language = language
        self.logger = logger or logging.getLogger(__name__)
        self.tts_cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'rumblex/tts'

    def synthesize(self, text):
        """Return audio to play; call from a worker, never the ROS executor."""
        digest = hashlib.sha256(f'{self.language}\0{text}'.encode()).hexdigest()
        self.tts_cache.mkdir(parents=True, exist_ok=True)
        filename = self.tts_cache / f'{digest}.mp3'
        if filename.is_file() and filename.stat().st_size:
            return filename
        with tempfile.TemporaryDirectory(prefix='.tts-', dir=self.tts_cache) as temp_dir:
            try:
                download = Path(temp_dir) / 'speech.mp3'
                gTTS(text=text, lang=self.language, timeout=10).save(str(download))
                download.replace(filename)
                return filename
            except Exception as exc:
                self.logger.warning(f'Online TTS failed; using espeak-ng: {exc}')
                filename = self.tts_cache / f'{digest}.wav'
                download = Path(temp_dir) / 'speech.wav'
                subprocess.run(
                    ['espeak-ng', '-v', self.language + '+f3', '-w', str(download), '--', text],
                    check=True, timeout=30, capture_output=True)
                download.replace(filename)
                return filename
