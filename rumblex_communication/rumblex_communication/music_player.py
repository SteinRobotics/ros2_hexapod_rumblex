"""Audio playback; completion is polled by the ROS executor."""

import logging
import os
from pathlib import Path

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame.mixer as mixer

from rumblex_communication import package_resource_path


class MusicPlayer:
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        self.sound_dir = package_resource_path('soundfiles')
        self._callback = None
        mixer.init()

    def play(self, filename, volume=0.5, cb=None):
        extension = Path(filename).suffix.lower().lstrip('.')
        if extension not in ('wav', 'mp3') or Path(filename).name != filename:
            raise ValueError(f'Unsupported audio filename: {filename}')
        self.play_file(self.sound_dir / extension / filename, volume, cb)

    def play_file(self, filepath, volume=0.5, cb=None):
        self.stop_playing()
        mixer.music.load(str(filepath))
        mixer.music.set_volume(volume)
        mixer.music.play()
        self._callback = cb

    def poll(self):
        if self._callback is not None and not self.is_busy():
            callback, self._callback = self._callback, None
            callback()

    def is_busy(self):
        return mixer.music.get_busy()

    def stop_playing(self):
        self._callback = None
        mixer.music.stop()

    def close(self):
        self.stop_playing()
        mixer.quit()
