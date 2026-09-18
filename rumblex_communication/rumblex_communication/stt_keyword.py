#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import os
import platform
import queue
import struct
from pathlib import Path
import threading
from threading import Thread

import sounddevice as sd

try:
    import pvporcupine
except ModuleNotFoundError:
    pvporcupine = None

from rumblex_communication import package_resource_path


class KeywordSpotter(Thread):
    def __init__(self, cb, keywords=None, sensitivities=None, access_key=None, keyword_dir=None, logger=None):
        super().__init__()
        self.cb = cb
        self.logger = logger or logging.getLogger(__name__)
        self.keywords = tuple(keywords or ("rumblex", "jamie"))
        self.sensitivities = tuple(sensitivities or (0.6,) * len(self.keywords))
        self.keyword_dir = Path(keyword_dir) if keyword_dir is not None else package_resource_path('models', 'porcupine')
        self._stop_requested = threading.Event()

        if pvporcupine is None:
            raise ModuleNotFoundError(
                "pvporcupine is required for offline keyword spotting. "
                "Install it with: pip install pvporcupine"
            )

        if len(self.sensitivities) != len(self.keywords):
            raise ValueError('sensitivities must match the number of keywords')

        self.access_key = access_key or self._load_access_key()
        self.keyword_paths = self._resolve_keyword_paths()

    def stop_listening(self):
        self._stop_requested.set()

    def stream_callback(self, indata, frames, time_info, status):
        if status:
            self.logger.warning(str(status))
        self.q.put(bytes(indata))

    def _load_access_key(self):
        env_access_key = os.environ.get('PICOVOICE_ACCESS_KEY', '').strip()
        if env_access_key:
            return env_access_key

        access_key_path = package_resource_path('keys', 'picovoice_access_key.txt')
        if access_key_path.exists():
            file_access_key = access_key_path.read_text(encoding='utf-8').strip()
            if file_access_key:
                return file_access_key

        raise FileNotFoundError(
            "Missing Picovoice access key. Set PICOVOICE_ACCESS_KEY or create "
            f"{access_key_path}."
        )

    def _resolve_keyword_paths(self):
        if not self.keyword_dir.exists():
            raise FileNotFoundError(
                f"Keyword directory not found: {self.keyword_dir}. "
                "Export Porcupine custom keywords there as .ppn files."
            )

        keyword_paths = []
        for keyword in self.keywords:
            candidates = sorted(
                path for path in self.keyword_dir.glob('*.ppn')
                if path.stem.lower().startswith(keyword.lower())
            )

            if not candidates:
                raise FileNotFoundError(
                    f"No Porcupine keyword file found for '{keyword}' in {self.keyword_dir}."
                )

            if len(candidates) > 1:
                platform_name = platform.system().lower()
                platform_matches = [path for path in candidates if platform_name in path.stem.lower()]
                if len(platform_matches) == 1:
                    candidates = platform_matches
                else:
                    matches = ', '.join(str(path.name) for path in candidates)
                    raise FileNotFoundError(
                        f"Multiple Porcupine keyword files matched '{keyword}': {matches}. "
                        "Keep one platform-specific .ppn file per keyword."
                    )

            keyword_paths.append(candidates[0])

        return keyword_paths

    def run(self):
        self.q = queue.Queue()
        detected_keyword = None
        porcupine = None

        try:
            porcupine = pvporcupine.create(
                access_key=self.access_key,
                keyword_paths=[str(path) for path in self.keyword_paths],
                sensitivities=list(self.sensitivities),
            )

            input_dev_num = sd.query_hostapis()[0]['default_input_device']
            if input_dev_num == -1:
                raise ValueError('No input device found, device number == -1')

            with sd.RawInputStream(
                samplerate=porcupine.sample_rate,
                blocksize=porcupine.frame_length,
                device=input_dev_num,
                dtype='int16',
                channels=1,
                callback=self.stream_callback,
            ):
                while not self._stop_requested.is_set():
                    try:
                        data = self.q.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    pcm = struct.unpack_from(f'{porcupine.frame_length}h', data)
                    keyword_index = porcupine.process(pcm)

                    if keyword_index >= 0:
                        detected_keyword = self.keywords[keyword_index]
                        self.logger.info(f"Detected keyword: {detected_keyword}")
                        break

        except Exception as exc:
            self.logger.error(f'{type(exc).__name__}: {exc}')
            return
        finally:
            if porcupine is not None:
                porcupine.delete()

        if detected_keyword is not None and not self._stop_requested.is_set():
            self.cb(detected_keyword)