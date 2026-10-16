from setuptools import setup
import glob
import os

package_name = 'rumblex_communication'
profile_configs = glob.glob('config/*/*.yaml')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/communication_launch.py']),
        *[(os.path.join('share', package_name, os.path.dirname(path)), [path])
          for path in profile_configs],
        (os.path.join('share', package_name, 'models'), glob.glob('models/*.onnx')),
    ],
    install_requires=[
        'setuptools',
        'openwakeword>=0.6.0,<0.7',
        'onnxruntime',
        'numpy',
        'SpeechRecognition',
        'google-cloud-speech',
        'gTTS',
        'sounddevice',
        'pygame',
        'PyAudio',
    ],
    zip_safe=True,
    maintainer='Christian Stein',
    maintainer_email='stein.robotics@gmail.com',
    description='Speech recognition, TTS, and audio I/O for the RumbleX hexapod platform',
    license='MIT',
    extras_require={'test': ['pytest', 'onnx']},
    entry_points={
        'console_scripts': [
            'node_communication = rumblex_communication.node_communication:main',
        ],
    },
)
