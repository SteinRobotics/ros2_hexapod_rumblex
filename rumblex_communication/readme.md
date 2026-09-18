# TTS
# STT Keyword spotting offline
## picovoice porcupine
Keyword module: `rumblex_communication/stt_keyword.py`

Expected runtime assets:
- Picovoice access key in `PICOVOICE_ACCESS_KEY` or `keys/picovoice_access_key.txt`
- Custom keyword files in `models/porcupine/` with one `.ppn` for `rumblex` and one for `jamie`

Install:
```
pip3 install pvporcupine --user --break-system-packages
```

TODO: change offline keword detection to:
https://picovoice.ai/platform/porcupine/

# STT
## openai
https://platform.openai.com/docs/guides/speech-to-text


# STT offline
ros-vosk/cfg/params.yaml:
vosk/model: vosk-model-small-en-us-0.15
vosk/sample_rate: 16000
vosk/blocksize: 16000

german models (added to .gitignore)
vosk-model-de-0.21
vosk-model-small-de-0.15

## ros package
git clone
https://github.com/alphacep/ros-vosk

## vosk-api
https://github.com/alphacep/vosk-api

## vosk models
https://alphacephei.com/vosk/models

## install
sudo apt install portaudio19-dev
sudo apt-get install espeak-ng espeak-ng-espeak mbrola
sudo apt install python3-pygame


pip3 install --user SpeechRecognition --break-system-packages

pip3 install sounddevice vosk pyttsx3 --user --break-system-packages

pip3 install SpeechRecognition pyaudio google-api-python-client google-cloud-speech oauth2client --user --break-system-packages

pip3 install gTTS pygobject --user --break-system-packages


_____
# create a virtual environment (not working)
```
cd ~/Workspace/colcon_rumblex
sudo apt install python3-virtualenv
virtualenv -p python3 ./venv
touch venv/COLCON_IGNORE
source ./venv/bin/activate
```

```
python3 -m pip install vosk pyttsx3 google-api-python-client google-cloud-speech oauth2client sounddevice SpeechRecognition pyyaml
```
