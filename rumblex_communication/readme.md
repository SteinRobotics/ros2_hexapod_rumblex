# Speech and wake-word recognition

`stt_keyword.py` is the default offline robot-name recognizer. The launch argument
`robot:=nox` (default) selects `models/nox.onnx`; `robot:=nira` selects
`models/nira.onnx`. Only the selected robot name activates the spoken response and
subsequent online speech recognition. Listening is enabled through `request_listening`
as before. The existing `STT_OFFLINE_ACTIVE` status identifies keyword listening.

Both robot profiles set `keyword_threshold: 0.5`: a model score of 50% or higher
triggers detection. This is a score threshold, not a measured recognition accuracy.
The microphone must support 16 kHz mono signed 16-bit PCM; inference uses 80 ms frames.

## Installation

In the ROS Python virtual environment install the ONNX backend:

```bash
python -m pip install 'openwakeword==0.6.0' --no-deps
python -m pip install onnxruntime numpy scipy scikit-learn requests tqdm sounddevice
python -c 'from openwakeword.utils import download_models; download_models(model_names=["nox", "nira"])'
```

The explicit dependency installation avoids openWakeWord 0.6.0's unused Linux
`tflite-runtime` dependency, which lacks wheels for newer Python versions. The
one-time download installs the shared audio feature models in openWakeWord's package
resources. The custom Nox and Nira models are included with this ROS package.
Runtime recognition is offline and requires no access key.

Build with the same Python interpreter and launch:

```bash
python -m colcon build --symlink-install --packages-select rumblex_communication
source install/local_setup.bash
ros2 launch rumblex_communication communication_launch.py robot:=nira
```

Online recognition still requires Google speech credentials. Audio playback and
microphone access require PortAudio and the existing audio dependencies.

## Tests

Install `pytest` and `onnx` in the environment above, then from this package run:

```bash
python -m pytest test
```

The keyword tests cover both model selections, the threshold boundary, cancellation,
model validation, and real ONNX inference on silence. Measuring recognition accuracy
requires labeled recordings of each name and background speech from the target microphones.

Backend documentation: https://github.com/dscripka/openWakeWord

## Audio lifecycle and deferred chat

Chat is currently unavailable. The legacy implementation has been removed;
`request_chat` logs a warning and publishes `OFF` so the brain does not wait forever.
A local chat backend will be implemented separately.

Speech synthesis runs in a background worker. The ROS timer processes worker results
and polls playback completion. Recognition starts only after the wake response has
finished playing. A new audio request cancels the previous request, and late worker
results are discarded. New capture/synthesis waits for previous workers to exit without
blocking ROS callbacks. Shutdown requests cancellation of all workers and allows a
shared two-second grace period; outstanding network calls may outlive that period in
daemon threads and cannot update the node afterward.

TTS caches audio under `$XDG_CACHE_HOME/rumblex/tts` (normally
`~/.cache/rumblex/tts`). Cache keys include the full text and language. Google TTS
uses a network timeout; if unavailable, `espeak-ng` creates a WAV file which uses the
same playback/completion path. Install the `espeak-ng` executable for this fallback.
Existing package-local TTS caches are no longer used and may be removed manually.

Google speech recognition respects an existing `GOOGLE_APPLICATION_CREDENTIALS`
setting. The legacy package-local `keys/SERVICE_ACCOUNT_KEY.JSON` is used only when
present and no credential environment setting exists.
