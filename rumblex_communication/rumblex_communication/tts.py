#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# online
# pip3 install gTTS

# offline
# https://wiki.ubuntuusers.de/eSpeak_NG/
# sudo apt-get install espeak-ng espeak-ng-espeak mbrola

import logging
import re
import subprocess
import tempfile
from pathlib import Path

from gtts import gTTS

from rumblex_communication import package_resource_path

class TextToSpeech():
    def __init__(self, music_player, language, logger=None):
        self.language = language
        self.music_player = music_player
        self.logger = logger or logging.getLogger(__name__)
        self.tts_cache = package_resource_path('tts_cache')

    def run(self, text, cb=None):
        text_shorten = re.sub("[^a-zA-Z0-9]+", "", text)[:60]
        text_file = text_shorten.lower() + ".mp3"
        filename = self.tts_cache / text_file

        try:
            self.tts_cache.mkdir(parents=True, exist_ok=True)
            if not filename.exists() or filename.stat().st_size == 0:
                tts = gTTS(text=text, lang=self.language)
                # Publish a cache entry only after the download succeeds. gTTS
                # opens its output before making the request, so a failed
                # request would otherwise leave a partial or empty MP3.
                with tempfile.TemporaryDirectory(prefix='.tts-', dir=self.tts_cache) as temp_dir:
                    download = Path(temp_dir) / text_file
                    tts.save(str(download))
                    download.replace(filename)

            self.music_player.play_file(str(filename))

        except Exception as e:
            self.logger.error(str(e))
            self.logger.info("Falling back to offline TTS (espeak-ng)")
            subprocess.call(["espeak-ng", "-v" + self.language + "+f3", text], stderr=subprocess.STDOUT)

        if cb:
            cb()

if __name__=="__main__":
    from rumblex_communication.music_player import MusicPlayer

    music_player = MusicPlayer()
    tts = TextToSpeech(music_player, "de")
    tts.run("Hallo wie geht es dir")

