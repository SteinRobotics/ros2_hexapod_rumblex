#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__copyright__ = "Copyright (C) 2025 Christian Stein"
__license__ = "MIT"

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

from rumblex_interfaces.msg import CommunicationStatus

from rumblex_communication.stt_offline import SpeechToTextOffline
from rumblex_communication.stt_online import SpeechToTextOnline
from rumblex_communication.tts import TextToSpeech
from rumblex_communication.music_player import MusicPlayer
# from rumblex_communication.chatbot import ChatBot


class NodeCommunication(Node):
    def __init__(self):
        super().__init__('node_communication')
        self.status = CommunicationStatus.OFF
        self.statusOld = CommunicationStatus.OFF
        self.music_file = ""
        self.text_to_talk = ""
        self.stt_offline = None
        self.stt_online = None

        self.declare_parameter('robot_names', rclpy.Parameter.Type.STRING_ARRAY)
        self.robot_names = self.get_parameter('robot_names').value
        self.get_logger().info('Starting Communication Node, hearing at robot names: "%s"' % self.robot_names)

        # create classes
        # TODO create a class for each service
        self.music_player = MusicPlayer(logger=self.get_logger())
        self.tts = TextToSpeech(self.music_player, "de", logger=self.get_logger())
        # self.chatbot = ChatBot()


        self.create_subscription(String, 'request_music', self.callback_music, 10)
        self.create_subscription(String, 'request_talking', self.callback_talking, 10)
        self.create_subscription(String, 'request_chat', self.callback_chatbot, 10)
        self.create_subscription(Bool, 'request_listening', self.callback_listening, 10)
        self.pubCommunicationStatus = self.create_publisher(CommunicationStatus, 'communication_status', 10)
        self.pubSpeechRecognitionOnline = self.create_publisher(String, 'speech_recognition_online', 10)

        self._set_status(CommunicationStatus.OFF)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    _STATUS_NAMES = {
        CommunicationStatus.OFF: "OFF",
        CommunicationStatus.PLAYING_MUSIC: "PLAYING_MUSIC",
        CommunicationStatus.STOPPING_MUSIC: "STOPPING_MUSIC",
        CommunicationStatus.TTS_ACTIVE: "TTS_ACTIVE",
        CommunicationStatus.STT_ONLINE_ACTIVE: "STT_ONLINE_ACTIVE",
        CommunicationStatus.STT_OFFLINE_ACTIVE: "STT_OFFLINE_ACTIVE",
        CommunicationStatus.CHAT_ACTIVE: "CHAT_ACTIVE",
    }

    def _set_status(self, status):
        self.status = status
        msg = CommunicationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = self.status
        self.pubCommunicationStatus.publish(msg)
        name = self._STATUS_NAMES.get(self.status, "UNKNOWN")
        self.get_logger().info(f'CommunicationStatus.{name}')

    # TODO is this really needed
    def timer_callback(self):
        if self.status == self.statusOld:
            return

        elif self.status == CommunicationStatus.STT_OFFLINE_ACTIVE:
            self.stt_offline = SpeechToTextOffline(self.robot_names, self.callback_robotname_recognized, logger=self.get_logger())
            self.stt_offline.start()

        elif self.status == CommunicationStatus.STT_ONLINE_ACTIVE:
            self.stt_online = SpeechToTextOnline(self.callback_done, logger=self.get_logger())
            self.stt_online.start()

        # elif self.status == CommunicationStatus.CHAT_ACTIVE:
        #     self.chatbot.setChatRequest(self.chat_question, self.callback_chat_talking)

        elif self.status == CommunicationStatus.OFF:
            self.join_all_threads()

        self.statusOld = self.status

    def join_all_threads(self):
        self.get_logger().info('join_all_threads: ' + str(self.statusOld))
        if self.statusOld == CommunicationStatus.STT_OFFLINE_ACTIVE and self.stt_offline is not None:
            self.stt_offline.stop_listening()
            self.stt_offline.join()
        elif self.statusOld == CommunicationStatus.STT_ONLINE_ACTIVE and self.stt_online is not None:
            self.stt_online.stop_listening()
            self.stt_online.join()

# callbacks triggered from  external ROS msgs
    def callback_music(self, msg):
        self.get_logger().info('callback_music "%s"' % msg.data)
        if msg.data == "STOP":
            self._set_status(CommunicationStatus.STOPPING_MUSIC)
            self.music_player.stop_playing(cb=self.callback_done)
            return

        self._set_status(CommunicationStatus.PLAYING_MUSIC)
        self.music_player.play(msg.data, cb=self.callback_done)

    def callback_talking(self, msg):
        self.get_logger().info('callback_talking "%s"' % msg.data)
        self.tts.run(msg.data, self.callback_done)
        self._set_status(CommunicationStatus.TTS_ACTIVE)

    def callback_chatbot(self, msg):
        self.get_logger().info('callback_chatbot "%s"' % msg.data)
        self.chat_question = msg.data
        self._set_status(CommunicationStatus.CHAT_ACTIVE)

    def callback_chat_talking(self, msg):
        self.get_logger().info('callback_chat_talking "%s"' % msg.data)
        self.text_to_talk = msg.data
        self._set_status(CommunicationStatus.TTS_ACTIVE)

    def callback_listening(self, msg):
        self.get_logger().info('callback_listening "%s"' % msg.data)
        if not msg.data:
            if self.stt_offline is not None:
                self.stt_offline.stop_listening()
            self._set_status(CommunicationStatus.OFF)
            return

        self._set_status(CommunicationStatus.STT_OFFLINE_ACTIVE)

# callbacks triggered internally
    def callback_robotname_recognized(self):
        self.get_logger().info('callback_robotname_recognized')

        if self.music_player.is_busy():
            self.music_player.stop_playing()

        list_of_answers = ["ja", "ja bitte", "wie kann ich helfen",
                           "was gibt es", "was kann ich tun", "ich bin hier", "was denn", "bereit", "ich bin da"]
        random_index = random.randrange(len(list_of_answers))
        self.tts.run(list_of_answers[random_index], self.callback_robotname_response_done)
        # self._set_status(CommunicationStatus.TTS_ACTIVE)

    def callback_robotname_response_done(self):
        self.get_logger().info('callback_robotname_response_done')
        self._set_status(CommunicationStatus.STT_ONLINE_ACTIVE)

    def callback_done(self, info=''):
        self.get_logger().info('callback_done')

        if self.status == CommunicationStatus.STT_ONLINE_ACTIVE:
            msg = String()
            msg.data = info
            self.pubSpeechRecognitionOnline.publish(msg)

        self._set_status(CommunicationStatus.OFF)



def main(args=None):
    rclpy.init(args=args)
    node_communication = NodeCommunication()

    rclpy.spin(node_communication)

    node_communication.join_all_threads()
    node_communication.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
