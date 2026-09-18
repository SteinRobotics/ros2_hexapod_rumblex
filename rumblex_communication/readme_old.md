
# ros-vosk

roslaunch ros_vosk ros_vosk.launch
rosrun ros_vosk vosk_node.py

<launch>
  <node name="vosk_engine" pkg="ros_vosk" type="vosk_node.py" respawn="true" output="screen" />
  <node name="tts_engine" pkg="ros_vosk" type="tts_engine.py" respawn="true" output="screen" />
  <rosparam file="$(find ros_vosk)/cfg/params.yaml" command="load" ns="" />
</launch>

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
pip3 install sounddevice
pip3 install vosk
pip3 install pyttsx3
sudo apt install portaudio19-dev


# OLD

sudo apt install ros-noetic-sound-play

http://wiki.ros.org/sound_play

scripts are at:
/opt/ros/noetic/lib/sound_play

'''
print('Usage: %s \'String to say.\'' % argv[0])
print('       %s < file_to_say.txt' % argv[0])

...

voice = 'voice_kal_diphone'
volume = 1.0

if len(argv) == 1:
    s = sys.stdin.read()
else:
    s = argv[1]

    if len(argv) > 2:
        voice = argv[2]
    if len(argv) > 3:
        volume = float(argv[3])
'''

___

Festival
sudo apt-get install festival festival-doc festival-freebsoft-utils

--language <string>
--batch

.feststialrc
'''
(set! voice_default voice_pc_diphone)
'''
Die Namen der installierten Stimmen findet man bei Installation aus den Paketquellen unter /usr/share/festival/voices/[SPRACHE]/

install voices:
https://ubuntuforums.org/showthread.php?t=677277
sudo apt-get install festlex-cmu

fetival german voice:
https://wiki.bsdforen.de/howto/deutsche_sprachausgabe_mit_festival.txt.html

____

espeak bzw. espeak-ng-espeak

https://wiki.ubuntuusers.de/eSpeak_NG/
'''
sudo apt-get install espeak-ng espeak-ng-espeak mbrola
'''
espeak-ng -vde "Text mit deutscher Sprache"

____

Pygame
https://www.pygame.org/wiki/GettingStarted
sudo apt-get install python3-pygame
or
python3 -m pip install -U pygame

