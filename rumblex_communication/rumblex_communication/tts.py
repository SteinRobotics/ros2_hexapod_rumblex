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

from gtts import gTTS

from rumblex_communication import package_resource_path
from rumblex_communication.music_player import MusicPlayer

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
            if not filename.exists():
                tts = gTTS(text=text, lang=self.language)
                tts.save(str(filename))

            self.music_player.play_file(str(filename))

        except Exception as e:
            self.logger.error(str(e))
            self.logger.info("Falling back to offline TTS (espeak-ng)")
            subprocess.call(["espeak-ng", "-v" + self.language + "+f3", text], stderr=subprocess.STDOUT)

        if cb:
            cb()

if __name__=="__main__":
    music_player = MusicPlayer()
    tts = TextToSpeech(music_player, "de")
    tts.run("Hallo wie geht es dir")


