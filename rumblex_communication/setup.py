from setuptools import setup
import os, glob

package_name = 'rumblex_communication'
profile_configs = glob.glob('config/*/*.yaml')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],

    data_files=[
        ('lib/' + package_name, ['rumblex_communication/chatbot.py', 'rumblex_communication/music_player.py', 'rumblex_communication/stt_keyword.py', 'rumblex_communication/stt_offline.py', 'rumblex_communication/stt_online.py', 'rumblex_communication/tts.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/communication_launch.py']),
        *[(os.path.join('share', package_name, os.path.dirname(path)), [path])
          for path in profile_configs],
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # ('target_directory_2', glob('nested_source_dir/**/*', recursive=True))
    ],
    install_requires=[
        'setuptools',
        'pvporcupine',
        ],
    zip_safe=True,
    maintainer='Christian Stein',
    maintainer_email='stein.robotics@gmail.com',
    description='Speech recognition, TTS, chatbot, and audio I/O for the RumbleX hexapod platform',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_communication = rumblex_communication.node_communication:main',
        ],
    },
)
