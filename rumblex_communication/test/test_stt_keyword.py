from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import pytest

from rumblex_communication.stt_keyword import KeywordSpotter


@pytest.mark.parametrize('robot', ['nox', 'nira'])
@pytest.mark.parametrize('score, expected', [(0.499, False), (0.5, True), (0.9, True)])
def test_threshold_and_robot_selection(robot, score, expected):
    callback = Mock()
    spotter = KeywordSpotter(callback, robot)
    spotter.stream_callback(bytes(2560), 1280, None, None)
    with patch('openwakeword.model.Model') as model, \
            patch('rumblex_communication.stt_keyword.sd.RawInputStream') as stream:
        def predict(pcm):
            assert pcm.dtype == np.int16
            assert pcm.shape == (1280,)
            if not expected:
                spotter.stop_listening()
            return {robot: score}
        model.return_value.predict.side_effect = predict
        spotter.run()
        model.assert_called_once_with(
            wakeword_models=[str(spotter.model_path)], inference_framework='onnx')
        stream.return_value.__exit__.assert_called_once()
    assert callback.call_count == int(expected)
    if expected:
        callback.assert_called_once_with()


def test_stop_prevents_detection():
    callback = Mock()
    spotter = KeywordSpotter(callback, 'nox')
    spotter.stop_listening()
    with patch('openwakeword.model.Model') as model, \
            patch('rumblex_communication.stt_keyword.sd.RawInputStream'):
        spotter.run()
    model.return_value.predict.assert_not_called()
    callback.assert_not_called()


@pytest.mark.parametrize('threshold', [-1, 0, 1.1, float('nan')])
def test_invalid_threshold(threshold):
    with pytest.raises(ValueError):
        KeywordSpotter(Mock(), 'nox', threshold=threshold)


def test_missing_model(tmp_path):
    with pytest.raises(FileNotFoundError):
        KeywordSpotter(Mock(), 'nira', model_dir=tmp_path)


@pytest.mark.parametrize('robot', ['nox', 'nira'])
def test_real_model_audio_pipeline(robot):
    from openwakeword.model import Model
    import onnx

    path = Path(__file__).parents[1] / 'models' / f'{robot}.onnx'
    onnx.checker.check_model(str(path))
    model = Model(wakeword_models=[str(path)], inference_framework='onnx')
    scores = [float(model.predict(np.zeros(1280, dtype=np.int16))[robot]) for _ in range(40)]
    assert all(np.isfinite(score) and 0 <= score < 0.5 for score in scores)
