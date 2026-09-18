#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Intergrated by Angelo Antikatzidis https://github.com/a-prototype/vosk_ros
# Source code based on https://github.com/alphacep/vosk-api/blob/master/python/example/test_microphone.py from VOSK's example code

# Tuned for the python flavor of VOSK: vosk-0.3.31
# If you do not have vosk then please install it by running $ pip3 install vosk
# If you have a previous version of vosk installed then update it by running $ pip3 install vosk --upgrade
# Tested on ROS Noetic & Melodic. Please advise the "readme" for using it with ROS Melodic

# This is a node that intergrates VOSK with ROS and supports a TTS engine to be used along with it
# When the TTS engine is speaking some words, the recognizer will stop listenning to the audio stream so it won't listen to it self :)

# It publishes to the topic speech_recognition/vosk_result a custom "speech_recognition" message
# It publishes to the topic speech_recognition/final_result a simple string
# It publishes to the topic speech_recognition/partial_result a simple string

#########
# origin take from the ros-vosk package from Angelo Antikatzidis (Apache License 2.0)
# very strongly adapted for robot "Nox" needs by Christian Stein
#########


import json
import logging
import queue
import threading
import time
from threading import Thread

import sounddevice as sd
import vosk

from rumblex_communication import package_resource_path


class SpeechToTextOffline(Thread):
    def __init__(self, robotnames, cb, logger=None):
        super().__init__()
        self.cb = cb
        self.robotnames = robotnames
        self._stop_requested = threading.Event()
        self.logger = logger or logging.getLogger(__name__)

        # change the name of the model to match the downloaded model's name
        model = 'vosk-model-small-en-us-0.15'

        self.model_dir = package_resource_path('models', model)

        if not self.model_dir.exists():
            raise FileNotFoundError(
                f"Could not find a model at: {self.model_dir}\n"
                "Please download a model for your language from https://alphacephei.com/vosk/models\n"
                "and unpack as 'model' in the folder /models."
            )

    def stop_listening(self):
        self._stop_requested.set()

    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            self.logger.warning(str(status))
        self.q.put(bytes(indata))

    def is_robotname_in_text(self, text):
        for robotname in self.robotnames:
            if robotname in text:
                return True
        return False

    def run(self):
        self.q = queue.Queue()

        input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if input_dev_num == -1:
            self.logger.error('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:

        samplerate = int(device_info['default_samplerate'])
        # rospy.set_param('vosk/sample_rate', samplerate)
        # rospy.set_param('vosk/blocksize', self.blocksize)

        model = vosk.Model(str(self.model_dir))

        try:
            # https://python-sounddevice.readthedocs.io/en/0.4.3/api/raw-streams.html#sounddevice.RawInputStream
            # with sd.RawInputStream(samplerate=samplerate, blocksize=16000, device=input_dev_num, dtype='int16',
            #                    channels=1, callback=self.stream_callback):
            with sd.RawInputStream(samplerate=samplerate, blocksize=2000, device=input_dev_num, dtype='int16',
                                   channels=1, callback=self.stream_callback):
                rec = vosk.KaldiRecognizer(model, samplerate)
                isRecognized = False
                result_text = ""

                while not self._stop_requested.is_set():
                    try:
                        data = self.q.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    if rec.AcceptWaveform(data):
                        # In case of final result
                        result = rec.FinalResult()
                        diction = json.loads(result)
                        lentext = len(diction["text"])

                        if lentext > 2:
                            result_text = diction["text"]
                            self.logger.info(result_text)
                            isRecognized = True
                        else:
                            isRecognized = False
                        # Resets current results so the recognition can continue from scratch
                        rec.Reset()

                    if isRecognized:
                        # rospy.loginfo("speech_recognition_offline: " + result_text)
                        time.sleep(0.1)
                        isRecognized = False
                        if self.is_robotname_in_text(result_text):
                            break

        except Exception as e:
            self.logger.error(f'{type(e).__name__}: {e}')
            return

        input_dev_num = None
        if not self._stop_requested.is_set():
            self.cb()
