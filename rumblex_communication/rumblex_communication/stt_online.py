import logging
import os
import threading
from threading import Thread

import speech_recognition as sr

from rumblex_communication import package_resource_path

class SpeechToTextOnline(Thread):
    def __init__(self, cb, logger=None):
        super().__init__(daemon=True)
        self.cb = cb
        self.logger = logger or logging.getLogger(__name__)
        self._stop_requested = threading.Event()

        credentials_file = package_resource_path('keys', 'SERVICE_ACCOUNT_KEY.JSON')
        if credentials_file.is_file():
            os.environ.setdefault("GOOGLE_APPLICATION_CREDENTIALS", str(credentials_file))

    def stop_listening(self):
        self._stop_requested.set()

    def run(self):
        r = sr.Recognizer()
        recog = "das habe ich nicht verstanden"

        try:
            with sr.Microphone() as source:
                self.logger.info("speech_recognition_online starts")
                r.adjust_for_ambient_noise(source)
                audio = r.listen(source, timeout=10, phrase_time_limit=6)
        except Exception as exc:
            self.logger.warning(f"Speech capture failed: {exc}")
            if self.cb and not self._stop_requested.is_set():
                self.cb(recog)
            return

        if self._stop_requested.is_set():
            return

        try:
            recog = r.recognize_google_cloud(audio, language_code='de-DE')
            self.logger.info("speech_recognition_online: " + recog)
        except sr.UnknownValueError as u:
            self.logger.warning(str(u))
            self.logger.warning("Google Cloud Speech Recognition could not understand audio")
        except Exception as e:
            self.logger.error(f"Could not request results from Google Cloud Speech Recognition service; {e}")

        if self.cb and not self._stop_requested.is_set():
            self.cb(recog)


if __name__ == "__main__":
    stt = SpeechToTextOnline(None)
    stt.run()
