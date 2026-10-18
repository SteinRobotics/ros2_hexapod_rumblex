#!/usr/bin/env bash
# Install the headless RumbleX runtime on Ubuntu Server 26.04 arm64.
# Run as the robot's regular login user; sudo is used for system changes only.
set -Eeuo pipefail
trap 'echo "Setup failed at line $LINENO. Fix the error above and rerun this script." >&2' ERR

usage() {
    echo "Usage: $0 <nox|nira>"
    echo "Run from a fresh login shell as the robot user, without sudo."
}
if [[ ${1:-} == --help || ${1:-} == -h ]]; then usage; exit 0; fi
if [[ $# != 1 || ! $1 =~ ^(nox|nira)$ ]]; then usage >&2; exit 2; fi
robot=$1
fail() { echo "ERROR: $*" >&2; exit 1; }
[[ $EUID != 0 ]] || fail "Run as the robot user, not root or sudo."
source /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 26.04 ]] || fail "Ubuntu 26.04 is required."
[[ $(dpkg --print-architecture) == arm64 ]] || fail "64-bit arm64 Ubuntu is required."
grep -aq 'Raspberry Pi 5' /proc/device-tree/model || fail "A Raspberry Pi 5 is required."
[[ -z ${ROS_DISTRO:-} && -z ${VIRTUAL_ENV:-} ]] || fail "Use a fresh shell without a sourced ROS workspace or virtual environment."
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
workspace="$HOME/Workspace/colcon_rumblex"
[[ $repo == "$workspace/src/"* ]] || fail "Clone this repository below $workspace/src first."
[[ -f /boot/firmware/config.txt ]] || fail "Missing Raspberry Pi firmware configuration."
robot_user=$(id -un)
[[ $robot_user =~ ^[a-z_][a-z0-9_-]*$ && $HOME != *[[:space:]]* ]] || fail "Unsupported login name or home path."
export LANG=C.UTF-8 LC_ALL=C.UTF-8
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
[[ $CMAKE_BUILD_PARALLEL_LEVEL =~ ^[1-9][0-9]*$ ]] || fail "Invalid CMAKE_BUILD_PARALLEL_LEVEL."
sudo -v
scratch=$(mktemp -d)
trap 'rm -rf -- "$scratch"' EXIT

# Stop an earlier installation before replacing files in its workspace.
if sudo systemctl cat autostart_ros2.service >/dev/null 2>&1; then
    sudo systemctl disable --now autostart_ros2.service
fi
if systemctl --user cat autostart_ros2.service >/dev/null 2>&1; then
    systemctl --user stop autostart_ros2.service
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends ca-certificates curl git locales software-properties-common
sudo add-apt-repository -y universe
sudo locale-gen en_US.UTF-8

# Use the official ROS repository configuration package, including key rotation.
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$scratch/release.json"
    apt_version=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$scratch/release.json")
    [[ $apt_version =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || fail "Unexpected ros2-apt-source release version."
    curl -fL --retry 3 "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${apt_version}/ros2-apt-source_${apt_version}.${VERSION_CODENAME}_all.deb" -o "$scratch/ros2-apt-source.deb"
    sudo dpkg -i "$scratch/ros2-apt-source.deb"
fi
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    ros-lyrical-ros-base python3-colcon-common-extensions python3-rosdep \
    build-essential cmake pkg-config python3-dev python3-venv python3-pip \
    python3-pygame python3-pil python3-pyaudio python3-lgpio python3-smbus \
    libi2c-dev i2c-tools libmagicenum-dev portaudio19-dev espeak-ng fonts-liberation \
    pipewire pipewire-pulse wireplumber dbus-user-session alsa-utils libpam-systemd

# Keep pip packages out of the externally managed Ubuntu Python installation.
# Colcon must run with this interpreter so generated node shebangs use the venv.
python3 -m venv --system-site-packages "$workspace/.venv"
touch "$workspace/.venv/COLCON_IGNORE"
source "$workspace/.venv/bin/activate"
python -m pip install --upgrade pip 'setuptools<80' wheel
python -m pip install 'cmake>=3.25,<4' \
    numpy scipy scikit-learn requests tqdm onnxruntime \
    SpeechRecognition google-cloud-speech gTTS sounddevice \
    adafruit-blinka adafruit-circuitpython-ssd1306 adafruit-circuitpython-ina228
# The unused tflite-runtime dependency has no wheels for current Ubuntu Python.
python -m pip install --no-deps 'openwakeword==0.6.0'
python -c 'from openwakeword.utils import download_models; download_models(model_names=["nox", "nira"])'

# ROS setup files are not compatible with Bash nounset.
set +u
source /opt/ros/lyrical/setup.bash
set -u

# Nira uses the SteinRobotics YDLIDAR fork over SSH.
# Keep the SDK checkout; update the driver to the published lyrical branch safely.
extra_packages=()
if [[ $robot == nira ]]; then
    vendor="$workspace/vendor"
    mkdir -p "$vendor"
    touch "$vendor/COLCON_IGNORE"
    if [[ ! -d $vendor/YDLidar-SDK ]]; then
        git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git "$vendor/YDLidar-SDK"
    fi
    cmake -S "$vendor/YDLidar-SDK" -B "$vendor/ydlidar-build" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$workspace/vendor/install" \
        -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=ON -DSWIG_FOUND=FALSE
    cmake --build "$vendor/ydlidar-build" --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
    cmake --install "$vendor/ydlidar-build"
    "$repo/scripts/update_ydlidar_driver.sh" "$workspace/src/ydlidar_ros2_driver"
    export CMAKE_PREFIX_PATH="$workspace/vendor/install:${CMAKE_PREFIX_PATH:-}"
    extra_packages+=(ydlidar_ros2_driver)
fi

# Resolve only runtime packages. GUI/simulation dependencies in shared manifests
# are intentionally skipped; pip dependencies above are satisfied in the venv.
packages=(rumblex_interfaces rumblex_utils rumblex_brain rumblex_movement
    rumblex_description rumblex_bringup rumblex_hmi rumblex_communication
    rumblex_teleop rumblex_lidar_1d rumblex_lidar_2d rumblex_perception rumblex_navigation)
paths=()
for package in "${packages[@]}"; do paths+=("$repo/$package"); done
for package in "${extra_packages[@]}"; do paths+=("$workspace/src/$package"); done
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update --rosdistro lyrical
skip_keys='rumblex_gazebo ros_gz_sim rviz2 joint_state_publisher_gui python-google-cloud-speech-pip python3-sounddevice-pip python-gTTS-pip magic_enum fonts-liberation'
if [[ $robot == nox ]]; then skip_keys+=' ydlidar_ros2_driver'; fi
# Disable recommended packages for rosdep's apt invocations as well.
printf 'APT::Install-Recommends "false";\n' > "$scratch/apt.conf"
sudo env APT_CONFIG="$scratch/apt.conf" /usr/bin/rosdep --sources-cache-dir "$HOME/.ros/rosdep/sources.cache" install \
    --from-paths "${paths[@]}" --ignore-src --rosdistro lyrical -y \
    --dependency-types build --dependency-types buildtool --dependency-types build_export \
    --dependency-types buildtool_export --dependency-types exec \
    --skip-keys "$skip_keys"
cd "$workspace"
python -m colcon build --base-paths "${paths[@]}" \
    --packages-select "${packages[@]}" "${extra_packages[@]}" \
    --symlink-install --executor sequential --event-handlers console_cohesion+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE="$workspace/.venv/bin/python" \
    -DPYTHON_EXECUTABLE="$workspace/.venv/bin/python"
set +u
source "$workspace/install/local_setup.bash"
set -u
python -c 'import rclpy, board, digitalio, adafruit_ssd1306, adafruit_ina228, lgpio, pygame, pyaudio, sounddevice, speech_recognition, google.cloud.speech, gtts, onnxruntime; from openwakeword.model import Model'
ros2 launch rumblex_bringup target_launch.py "robot:=$robot" --show-args

# Append a dedicated firmware block without replacing Ubuntu's configuration.
if ! grep -q '^# BEGIN RUMBLEX HARDWARE$' /boot/firmware/config.txt; then
    if [[ ! -e /boot/firmware/config.txt.before-rumblex ]]; then
        sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.before-rumblex
    fi
    printf '\n# BEGIN RUMBLEX HARDWARE\n[all]\ndtparam=i2c_arm=on\n# END RUMBLEX HARDWARE\n' | sudo tee -a /boot/firmware/config.txt >/dev/null
fi
printf 'i2c-dev\n' | sudo tee /etc/modules-load.d/rumblex.conf >/dev/null
for group in dialout audio video input i2c gpio; do
    sudo groupadd -f "$group"
    sudo usermod -aG "$group" "$robot_user"
done
sudo install -m 0644 "$repo/rumblex_doc/setup_raspberry_pi/99-rumblex.rules" /etc/udev/rules.d/99-rumblex.rules
sudo udevadm control --reload-rules

# Permit exactly the shutdown command used by the hardware HMI.
printf '%s ALL=(root) NOPASSWD: /usr/sbin/shutdown -h now\n' "$robot_user" > "$scratch/sudoers"
sudo visudo -cf "$scratch/sudoers"
sudo install -m 0440 "$scratch/sudoers" "/etc/sudoers.d/rumblex-$robot_user"

# A lingering user manager supplies PipeWire and hardware access without a GUI.
mkdir -p "$HOME/.config/rumblex" "$HOME/.config/systemd/user" "$HOME/.local/bin"
printf 'RUMBLEX_ROBOT=%s\n' "$robot" > "$HOME/.config/rumblex/robot.env"
install -m 0755 "$repo/rumblex_bringup/autostart/start_ros2.sh" "$HOME/.local/bin/rumblex-start"
install -m 0644 "$repo/rumblex_bringup/autostart/autostart_ros2.service" "$HOME/.config/systemd/user/autostart_ros2.service"
sudo loginctl enable-linger "$robot_user"
systemctl --user daemon-reload
systemctl --user enable pipewire.socket pipewire-pulse.socket wireplumber.service autostart_ros2.service

if [[ ! -f $repo/rumblex_communication/models/$robot.onnx ]]; then
    echo "Wake-word recognition needs the private model: $repo/rumblex_communication/models/$robot.onnx"
    echo "Copy it there and rerun the build before enabling listening."
fi
printf '\nSetup complete for %s. Reboot to apply I2C and group permissions and start the robot.\n' "$robot"
echo 'After reboot: systemctl --user status autostart_ros2.service'
echo 'Logs: journalctl --user -u autostart_ros2.service -b -f'
echo 'Online speech recognition additionally requires Google credentials (see setup documentation).'
