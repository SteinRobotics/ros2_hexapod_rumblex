"""Speech synthesis with a writable, content-addressed audio cache."""

import hashlib
import json
import logging
from pathlib import Path
import subprocess
import tempfile

from gtts import gTTS

from rumblex_communication import package_resource_path


class TextToSpeech:
    def __init__(self, language, logger=None):
        self.language = language
        self.logger = logger or logging.getLogger(__name__)
        self.tts_cache = package_resource_path('tts_cache')

    def synthesize(self, text):
        """Return audio to play; call from a worker, never the ROS executor."""
        digest = hashlib.sha256(f'{self.language}\0{text}'.encode()).hexdigest()
        self.tts_cache.mkdir(parents=True, exist_ok=True)
        filename = self.tts_cache / f'{digest}.mp3'
        if filename.is_file() and filename.stat().st_size:
            self._write_metadata(filename, text, 'gtts')
            return filename
        with tempfile.TemporaryDirectory(prefix='.tts-', dir=self.tts_cache) as temp_dir:
            try:
                download = Path(temp_dir) / 'speech.mp3'
                gTTS(text=text, lang=self.language, timeout=10).save(str(download))
            except Exception as exc:
                self.logger.warning(f'Online TTS failed; using espeak-ng: {exc}')
                filename = self.tts_cache / f'{digest}.wav'
                download = Path(temp_dir) / 'speech.wav'
                subprocess.run(
                    ['espeak-ng', '-v', self.language + '+f3', '-w', str(download), '--', text],
                    check=True, timeout=30, capture_output=True)
                engine = 'espeak-ng'
            else:
                engine = 'gtts'
            download.replace(filename)
            self._write_metadata(filename, text, engine)
            return filename

    def _write_metadata(self, filename, text, engine):
        """Keep each audio format identifiable, including fallback WAVs."""
        metadata = {
            'text': text,
            'language': self.language,
            'engine': engine,
            'audio_file': filename.name,
        }
        if engine == 'espeak-ng':
            metadata['voice'] = self.language + '+f3'
        # Atomic replacement keeps concurrent readers from seeing partial JSON.
        with tempfile.TemporaryDirectory(prefix='.tts-meta-', dir=self.tts_cache) as temp_dir:
            temporary = Path(temp_dir) / 'metadata.json'
            temporary.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            temporary.replace(filename.with_suffix(filename.suffix + '.json'))
