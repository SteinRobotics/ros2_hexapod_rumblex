    4  sudo dpkg-reconfigure keyboard-configuration
    5  reboot
    6  sudo nano /etc/netplan/50-cloud-init.yaml
    7  reboot
   13  sudo netplan --debug try
   15  sudo netplan --debug generate
   16  sudo netplan --debug apply
   18  reboot
   27  networkctl
   28  ping google.de
   29  sudo apt update
   30  sudo apt upgrade
   31  sudo apt autoremove
   32  sudo apt clean
   49  cd .ssh/
   57  ssh-keygen
   58  ls ~/.ssh/
   59  cat  ~/.ssh/id_ed25519.pub
   60  ssh alexej@lapi
   62  scp /home/ubuntu/.ssh/id_ed25519.pub alexej@lapi:/home/alexej/
   72  sudo apt install python-is-python3
   75  sudo apt install python3-pip
   80  scp alexej@lapi:~/.ssh/id_rsa.pub ~/
   83  cat id_rsa.pub  > ~/.ssh/authorized_keys
   86  sudo apt install -y i2c-tools python3-smbus
   88  sudo i2cdetect -y 1
   91  journalctl
   93  pip3 install adafruit-circuitpython-bno055 --break-system-packages
   95  sudo apt install python3-libgpiod
  100  cat /etc/udev/rules.d/90-gpio.rules
  103  sudo touch /etc/udev/rules.d/90-gpio.rules
  105  sudo nano /etc/udev/rules.d/90-gpio.rules
  106  sudo groupadd -f --system gpio
  107  sudo usermod -a -G gpio ubuntu
  118  sudo groupadd -f --system i2c
  119  sudo usermod -a -G i2c ubuntu
  125  ifconfig
  126  ip addr
  129  groups
  148  git clone git@bitbucket.org:steinrobotics/ros2_rumblex.git
  153  sudo vcgencmd measure_temp
  155  journalctl
  157  sudo dmesg -w
  163  sudo apt-get install pulseaudio
  168  sudo apt install alsa-base
  175  sudo alsamixer
  182  pip3 install terminalio --break-system-packages
  185  pip3 install adafruit-circuitpython-ssd1306 --break-system-packages
  190  sudo usermod -aG audio ubuntu
  191  sudo aplay /usr/share/sounds/alsa/Front_Center.wav


____

place 90-gpio.rules at /etc/udev/rules.d/90-gpio.rules

____

pactl set-sink-volume alsa_output.usb-Jieli_Technology_UACDemoV1.0_503581119377601F-00.analog-stereo 70%
pactl set-source-volume alsa_input.usb-C-Media_Electronics_Inc._USB_PnP_Sound_Device-00.analog-mono 100%

____

sudo visudo
rumblex ALL=(ALL) NOPASSWD: /sbin/shutdown, /sbin/restart, /bin/systemctl start autostart_ros2.service, /bin/systemctl stop autostart_ros2.service