"""Audio playback; completion is polled by the ROS executor."""

import logging
import math
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
        self._background_channel = None
        mixer.init()

    def play(self, filename, volume=0.5, cb=None, *, background=False):
        extension = Path(filename).suffix.lower().lstrip('.')
        if extension not in ('wav', 'mp3') or Path(filename).name != filename:
            raise ValueError(f'Unsupported audio filename: {filename}')
        filepath = self.sound_dir / extension / filename
        if not background:
            self.play_file(filepath, volume, cb)
            return

        # A separate mixer channel lets announcements use the streamed audio
        # player without interrupting this track.
        sound = mixer.Sound(str(filepath))
        sound.set_volume(volume)
        self.stop_background()
        self._background_channel = sound.play()
        if self._background_channel is None:
            raise RuntimeError('No audio channel available for background music')

    def stop_background(self):
        if self._background_channel is not None:
            self._background_channel.stop()
            self._background_channel = None

    def fade_background(self, duration_s):
        """Fade just the background channel; SDL performs the ramp asynchronously."""
        if not math.isfinite(duration_s) or not 0 <= duration_s <= 60:
            raise ValueError('Background fade duration must be 0-60 seconds')
        if duration_s == 0:
            self.stop_background()
        elif self._background_channel is not None:
            self._background_channel.fadeout(max(1, round(duration_s * 1000)))

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
        self.stop_background()
        mixer.quit()
