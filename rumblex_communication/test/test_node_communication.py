import queue
from unittest.mock import Mock

from rumblex_communication.node_communication import NodeCommunication
from rumblex_interfaces.msg import CommunicationStatus


def make_node():
    node = NodeCommunication.__new__(NodeCommunication)
    node._generation = 0
    node._events = queue.Queue()
    node._workers = []
    node._pending = None
    node._closed = False
    node.music_player = Mock()
    node.get_logger = Mock(return_value=Mock())
    node._set_status = lambda status: setattr(node, 'status', status)
    node.status = CommunicationStatus.OFF
    return node


def test_chat_request_completes_without_chat_backend():
    node = make_node()
    node.callback_chatbot(Mock(data='Hallo'))
    assert node.status == CommunicationStatus.OFF
    node.get_logger().warning.assert_called_once()


def test_cancel_discards_queued_results():
    node = make_node()
    callback = Mock()
    post = node._post(callback)
    node._cancel()
    post('late response')
    node.timer_callback()
    callback.assert_not_called()


def test_wake_response_waits_for_audio_completion():
    node = make_node()
    node._speak = Mock()
    node.callback_robotname_recognized()
    done = node._speak.call_args.args[1]
    node._play_speech('response.mp3', done)
    assert node.status == CommunicationStatus.OFF
    node.music_player.play_file.call_args.kwargs['cb']()
    assert node.status == CommunicationStatus.STT_ONLINE_ACTIVE
    assert node._pending is not None


def test_pending_capture_waits_for_previous_worker():
    node = make_node()
    worker = Mock()
    worker.is_alive.return_value = True
    node._workers = [worker]
    pending = Mock()
    node._pending = pending
    node.timer_callback()
    pending.assert_not_called()
    worker.is_alive.return_value = False
    node.timer_callback()
    pending.assert_called_once_with()


def test_shutdown_stops_and_joins_all_workers():
    node = make_node()
    node.timer = Mock()
    node._workers = [Mock(), Mock()]
    for worker in node._workers:
        worker.is_alive.return_value = False
    node.join_all_threads()
    for worker in node._workers:
        worker.stop_listening.assert_called_once()
        worker.join.assert_called_once()
    node.music_player.close.assert_called_once()


def test_talking_sets_active_before_starting_worker():
    node = make_node()
    node.callback_talking(Mock(data='Hallo'))
    assert node.status == CommunicationStatus.TTS_ACTIVE
    assert node._pending is not None


def test_background_music_does_not_interrupt_foreground_request():
    node = make_node()
    node.callback_talking(Mock(data='Hallo'))
    pending = node._pending
    generation = node._generation
    node.music_player.reset_mock()
    node.callback_background_music(Mock(data='party.mp3'))
    node.music_player.play.assert_called_once_with('party.mp3', background=True)
    node.music_player.stop_playing.assert_not_called()
    assert node._pending is pending
    assert node._generation == generation
    assert node.status == CommunicationStatus.TTS_ACTIVE


def test_foreground_cancellation_preserves_background_but_explicit_stop_ends_it():
    node = make_node()
    node.callback_background_music(Mock(data='party.mp3'))
    node.callback_talking(Mock(data='Hallo'))
    node._cancel()
    node.music_player.stop_background.assert_not_called()
    node.callback_music(Mock(data='STOP'))
    node.music_player.stop_background.assert_called_once_with()
    assert node.status == CommunicationStatus.OFF


def test_background_music_failure_does_not_finish_foreground_request():
    node = make_node()
    node.status = CommunicationStatus.STT_OFFLINE_ACTIVE
    node.music_player.play.side_effect = RuntimeError('audio error')
    node.callback_background_music(Mock(data='party.mp3'))
    node.get_logger().error.assert_called_once()
    assert node.status == CommunicationStatus.STT_OFFLINE_ACTIVE


def test_background_fade_preserves_foreground_request_and_handles_failure():
    node = make_node()
    node.callback_talking(Mock(data='Hallo'))
    pending, generation = node._pending, node._generation
    node.music_player.reset_mock()
    node.callback_background_music_fade(Mock(data=4.0))
    node.music_player.fade_background.assert_called_once_with(4.0)
    node.music_player.stop_playing.assert_not_called()
    node.music_player.stop_background.assert_not_called()
    assert node._pending is pending
    assert node._generation == generation
    assert node.status == CommunicationStatus.TTS_ACTIVE
    node.music_player.fade_background.side_effect = ValueError('invalid fade')
    node.callback_background_music_fade(Mock(data=-1.0))
    node.get_logger().error.assert_called_once()
    assert node.status == CommunicationStatus.TTS_ACTIVE
