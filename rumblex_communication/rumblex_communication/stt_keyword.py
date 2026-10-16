#!/usr/bin/env python3
"""Offline robot-name detection using openWakeWord ONNX models."""

import logging
from pathlib import Path
import queue
import threading

import numpy as np
import sounddevice as sd

from rumblex_communication import package_resource_path


class KeywordSpotter(threading.Thread):
    SAMPLE_RATE = 16000
    FRAME_LENGTH = 1280  # openWakeWord consumes 80 ms of mono PCM per prediction.

    def __init__(self, cb, robot, threshold=0.5, model_dir=None, logger=None):
        super().__init__(daemon=True)
        if robot not in ('nox', 'nira'):
            raise ValueError('robot must be nox or nira')
        if not 0 < threshold <= 1:
            raise ValueError('threshold must be greater than zero and at most one')
        self.cb = cb
        self.robot = robot
        self.threshold = threshold
        self.logger = logger or logging.getLogger(__name__)
        directory = Path(model_dir) if model_dir is not None else package_resource_path('models')
        self.model_path = directory / f'{robot}.onnx'
        if not self.model_path.is_file():
            raise FileNotFoundError(f'Wake-word model not found: {self.model_path}')
        self._stop_requested = threading.Event()
        self.q = queue.Queue(maxsize=32)

    def stop_listening(self):
        self._stop_requested.set()

    def stream_callback(self, indata, frames, time_info, status):
        if status:
            self.logger.warning(str(status))
        try:
            self.q.put_nowait(bytes(indata))
        except queue.Full:
            pass

    def run(self):
        detected = False
        try:
            from openwakeword.model import Model

            model = Model(wakeword_models=[str(self.model_path)], inference_framework='onnx')
            with sd.RawInputStream(
                samplerate=self.SAMPLE_RATE,
                blocksize=self.FRAME_LENGTH,
                dtype='int16',
                channels=1,
                callback=self.stream_callback,
            ):
                while not self._stop_requested.is_set():
                    try:
                        data = self.q.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    predictions = model.predict(np.frombuffer(data, dtype=np.int16))
                    if float(predictions[self.robot]) >= self.threshold:
                        detected = True
                        break
        except Exception as exc:
            self.logger.error(f'{type(exc).__name__}: {exc}')
            return

        # Release the microphone before the callback starts the spoken response.
        if detected and not self._stop_requested.is_set():
            self.logger.info(f'Detected keyword: {self.robot}')
            self.cb()
