#!/usr/bin/env python3
"""Coordinate audio services; all state changes occur on the ROS executor."""

import queue
import random
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rumblex_interfaces.msg import CommunicationStatus
from rumblex_communication.music_player import MusicPlayer
from rumblex_communication.stt_keyword import KeywordSpotter
from rumblex_communication.stt_online import SpeechToTextOnline
from rumblex_communication.tts import TextToSpeech


class NodeCommunication(Node):
    def __init__(self):
        super().__init__('node_communication')
        self.status = CommunicationStatus.OFF
        self.robot = self.declare_parameter('robot', rclpy.Parameter.Type.STRING).value
        self.keyword_threshold = self.declare_parameter('keyword_threshold', 0.5).value
        self.music_player = MusicPlayer(logger=self.get_logger())
        self.tts = TextToSpeech('de', logger=self.get_logger())
        self._events = queue.Queue()
        self._generation = 0
        self._workers = []
        self._pending = None
        self._closed = False

        self.create_subscription(String, 'request_music', self.callback_music, 10)
        self.create_subscription(String, 'request_talking', self.callback_talking, 10)
        self.create_subscription(String, 'request_chat', self.callback_chatbot, 10)
        self.create_subscription(Bool, 'request_listening', self.callback_listening, 10)
        self.pubCommunicationStatus = self.create_publisher(CommunicationStatus, 'communication_status', 10)
        self.pubSpeechRecognitionOnline = self.create_publisher(String, 'speech_recognition_online', 10)
        self._set_status(CommunicationStatus.OFF)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def _set_status(self, status):
        self.status = status
        msg = CommunicationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = status
        self.pubCommunicationStatus.publish(msg)

    def _post(self, callback):
        generation = self._generation
        return lambda *args: self._events.put((generation, callback, args))

    def _cancel(self):
        self._generation += 1
        self._pending = None
        for worker in self._workers:
            if hasattr(worker, 'stop_listening'):
                worker.stop_listening()
        self.music_player.stop_playing()

    def timer_callback(self):
        if self._closed:
            return
        self._workers = [worker for worker in self._workers if worker.is_alive()]
        while True:
            try:
                generation, callback, args = self._events.get_nowait()
            except queue.Empty:
                break
            if generation == self._generation:
                callback(*args)
        self.music_player.poll()
        # Wait without blocking the executor until previous capture/synthesis exits.
        if self._pending is not None and not self._workers:
            pending, self._pending = self._pending, None
            pending()

    def _start_worker(self, worker):
        self._workers.append(worker)
        worker.start()

    def _start_recognition(self, keyword):
        try:
            if keyword:
                worker = KeywordSpotter(
                    self._post(self.callback_robotname_recognized), self.robot,
                    threshold=self.keyword_threshold, logger=self.get_logger())
            else:
                worker = SpeechToTextOnline(self._post(self.callback_done), logger=self.get_logger())
            # Ensure initialization/device failures also complete the request.
            finished = self._post(self._recognition_finished)
            run = worker.run

            def run_and_finish():
                try:
                    run()
                finally:
                    finished()

            worker.run = run_and_finish
            self._start_worker(worker)
        except Exception as exc:
            self.get_logger().error(f'Cannot start recognition: {exc}')
            self.callback_done()

    def _recognition_finished(self):
        if self.status in (CommunicationStatus.STT_OFFLINE_ACTIVE, CommunicationStatus.STT_ONLINE_ACTIVE):
            self._set_status(CommunicationStatus.OFF)

    def _speak(self, text, done):
        self._set_status(CommunicationStatus.TTS_ACTIVE)
        ready = self._post(lambda path: self._play_speech(path, done))
        failed = self._post(self._audio_failed)

        def synthesize():
            try:
                ready(self.tts.synthesize(text))
            except Exception as exc:
                failed(str(exc))

        self._pending = lambda: self._start_worker(threading.Thread(target=synthesize, daemon=True))

    def _play_speech(self, path, done):
        try:
            self.music_player.play_file(path, cb=done)
        except Exception as exc:
            self._audio_failed(str(exc))

    def _audio_failed(self, error):
        self.get_logger().error(f'Audio request failed: {error}')
        self._set_status(CommunicationStatus.OFF)

    def callback_music(self, msg):
        self._cancel()
        if msg.data == 'STOP':
            self._set_status(CommunicationStatus.OFF)
            return
        self._set_status(CommunicationStatus.PLAYING_MUSIC)
        try:
            self.music_player.play(msg.data, cb=self.callback_done)
        except Exception as exc:
            self._audio_failed(str(exc))

    def callback_talking(self, msg):
        self._cancel()
        self._speak(msg.data, self.callback_done)

    def callback_chatbot(self, msg):
        self.get_logger().warning('Chat is unavailable; implementation is deferred.')
        self._cancel()
        self._set_status(CommunicationStatus.OFF)

    def callback_listening(self, msg):
        self._cancel()
        if msg.data:
            self._set_status(CommunicationStatus.STT_OFFLINE_ACTIVE)
            self._pending = lambda: self._start_recognition(keyword=True)
        else:
            self._set_status(CommunicationStatus.OFF)

    def callback_robotname_recognized(self):
        answers = ['ja', 'ja bitte', 'wie kann ich helfen', 'was gibt es', 'ich bin hier']
        self._speak(random.choice(answers), self.callback_robotname_response_done)

    def callback_robotname_response_done(self):
        self._set_status(CommunicationStatus.STT_ONLINE_ACTIVE)
        self._pending = lambda: self._start_recognition(keyword=False)

    def callback_done(self, info=''):
        if self.status == CommunicationStatus.STT_ONLINE_ACTIVE:
            msg = String()
            msg.data = info
            self.pubSpeechRecognitionOnline.publish(msg)
        self._set_status(CommunicationStatus.OFF)

    def join_all_threads(self):
        self._closed = True
        self.timer.cancel()
        self._cancel()
        deadline = time.monotonic() + 2.0
        for worker in self._workers:
            worker.join(timeout=max(0, deadline - time.monotonic()))
            if worker.is_alive():
                self.get_logger().warning('Audio worker still exiting; late results will be discarded.')
        self.music_player.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NodeCommunication()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.join_all_threads()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
