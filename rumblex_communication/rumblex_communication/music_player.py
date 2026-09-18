#!/usr/bin/env python3

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import logging
import os
# Hide the pygame support prompt
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import time

import pygame.mixer as mixer

from rumblex_communication import package_resource_path


class MusicPlayer():
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        self.sound_dir = package_resource_path('soundfiles')
        mixer.init()
        mixer.music.set_volume(0.5)  #between 0.0 and 1.0
        self.channel = None

    def play(self, filename, volume=0.5, cb=None):
        mixer.music.set_volume(volume)  #between 0.0 and 1.0
        _, dot, ext = filename.rpartition(".")
        ext = ext.lower() if dot else ""
        suffix = f".{ext}" if ext else ""
        if ext not in ("wav", "mp3"):
            self.logger.warning(f"Unsupported audio format: {suffix or 'unknown'}")
            return

        file_plus_path = self.sound_dir / ext / filename
        if not file_plus_path.exists():
            self.logger.warning(f"File {file_plus_path} does not exist!")
            return

        self.play_soundfile(file_plus_path)

        if cb:
            cb()

    def play_file(self, filepath, volume=0.5, cb=None):
        """Play a sound file from an absolute or arbitrary path."""
        mixer.music.set_volume(volume)
        self.play_soundfile(filepath)
        if cb:
            cb()

    def play_random_music(self):
        self.play("musicfox_hot_dogs_for_breakfast.mp3")

    def is_busy(self):
        return mixer.music.get_busy()
    

    # def play_stoppable(self, soundfile):
    #     soundObj = mixer.Sound(soundfile)
    #     self.channel = soundObj.play()

    #     # TODO check if the blocking behavior is desired here
    #     while self.channel.get_busy() and not self.is_stop_requested:
    #         time.sleep(0.1)
    #     mixer.music.stop() # not sure this one is correct

    def play_soundfile(self, soundfile):
        mixer.music.load(soundfile)
        # mixer.music.play(-1, 0.0) # -1: endless, 0.0: from beginning
        mixer.music.play()

    def stop_playing(self, cb=None):
        mixer.music.stop()
        if cb:
            cb()

if __name__ == "__main__":
    player = MusicPlayer()
    player.play_random_music()
    timepoint = time.time()
    while player.is_busy():
        time.sleep(0.1)
        if time.time() - timepoint > 5.0:
            print("Stopping playback after 5s")
            player.stop_playing()
            break
    print("done")

