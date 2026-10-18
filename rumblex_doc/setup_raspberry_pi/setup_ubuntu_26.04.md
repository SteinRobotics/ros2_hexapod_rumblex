# Raspberry Pi 5: Ubuntu Server 26.04, ROS 2 Lyrical

Das Setup setzt Ubuntu **Server 26.04 (64 Bit/arm64)**, funktionierendes WLAN,
Internetzugang und einen normalen Benutzer mit sudo-Rechten voraus. Das Repository
muss bereits unter `~/Workspace/colcon_rumblex/src/` liegen. Der Benutzer, der das
Skript ausführt, wird auch der Benutzer des Roboterdienstes.

Aus einer frischen Shell ohne aktivierte ROS-Installation oder Python-venv:

```bash
cd ~/Workspace/colcon_rumblex/src/ros2_hexapod_rumblex
./scripts/setup_raspberry_pi.sh nira
# Für Nox stattdessen: ./scripts/setup_raspberry_pi.sh nox
sudo reboot
```

Das Skript selbst **ohne sudo** aufrufen. Es fordert sudo für Systemänderungen an.
Es installiert ROS 2 Lyrical `ros-base`, Build-Werkzeuge, die Runtime-Abhängigkeiten,
Hardwaretreiber und Audio ohne Desktop, RViz oder Gazebo. Python-Pakete werden in
`~/Workspace/colcon_rumblex/.venv` mit Zugriff auf Ubuntu-/ROS-Pakete installiert;
die System-Python-Installation bleibt durch pip unverändert. openWakeWord verwendet
ONNX statt der nicht benötigten TFLite-Abhängigkeit. CMake 3.x wird für das ältere
YDLIDAR-SDK in der venv bereitgestellt.

Der Build erfolgt paketweise mit maximal zwei parallelen Compilerprozessen.
Bei wenig RAM kann `CMAKE_BUILD_PARALLEL_LEVEL=1 ./scripts/setup_raspberry_pi.sh nira`
verwendet werden. Nira erhält zusätzlich das YDLIDAR-SDK unter `vendor/` und den
ROS-Treiber aus dessen `humble`-Branch unter `src/ydlidar_ros2_driver`. Vorhandene
Vendor-Checkouts werden bei Wiederholung weiterverwendet, nicht zurückgesetzt oder
aktualisiert. Der mitgelieferte Patch `scripts/ydlidar_lyrical.patch` portiert
den Treiber auf C++23 und die CMake-Targets von Lyrical. Er wird genau einmal
angewendet; bei inkompatiblen lokalen Änderungen bricht das Setup ab.
Bestehende Build-Verzeichnisse müssen zu diesem Pi, Ubuntu und ROS
Lyrical passen; keine Build-/Install-Artefakte vom Entwicklungsrechner kopieren.

Das Setup aktiviert I²C, installiert udev-Regeln für GPIO/I²C/USB-Serial/Joystick,
vergibt Hardwaregruppen und erlaubt dem HMI ausschließlich
`sudo shutdown -h now` ohne Passwort. Die Firmwaredatei wird vor der ersten
Ergänzung als `config.txt.before-rumblex` gesichert. WLAN wird nicht verändert.

## Autostart und Bedienung

Der Dienst wird als **systemd-Benutzerdienst** installiert. Linger startet die
Benutzersitzung bereits beim Booten; PipeWire, PulseAudio-Kompatibilität und
WirePlumber stehen damit auch ohne GUI-Anmeldung bereit. Ein vorhandener alter
systemweiter `autostart_ros2.service` wird deaktiviert, damit der Roboter nicht
zweimal startet. Ein laufender Roboterdienst wird vor dem Build gestoppt; auch
bei einem Fehler bleibt er gestoppt, bis der Fehler behoben ist.

Der erste Start erfolgt nach dem manuellen Neustart, damit Gruppenrechte und I²C
wirksam sind. Dabei startet die Hardware einschließlich Servosteuerung und Relais.

```bash
systemctl --user status autostart_ros2.service
journalctl --user -u autostart_ros2.service -b -f
systemctl --user stop autostart_ros2.service
systemctl --user restart autostart_ros2.service
systemctl --user disable --now autostart_ros2.service
```

`~/.config/rumblex/robot.env` enthält `RUMBLEX_ROBOT=nira` bzw. `nox`. Der Launcher
ruft `ros2 launch rumblex_bringup target_launch.py robot:=nira` entsprechend dieser
Auswahl auf. `robot_nira` ist kein Launch-Argument. Für einen Profilwechsel das
Setup erneut mit dem anderen Roboternamen ausführen, damit insbesondere Niras
zusätzlicher Lidar-Treiber installiert wird. Navigation und automatisches Zuhören
bleiben bei den Standardwerten des Target-Launches (`false`).

Für manuelle ROS-Kommandos:

```bash
source /opt/ros/lyrical/setup.bash
source ~/Workspace/colcon_rumblex/.venv/bin/activate
source ~/Workspace/colcon_rumblex/install/local_setup.bash
ros2 node list
wpctl status
ls -l /dev/i2c-1 /dev/gpiochip* /dev/serial/by-id/
```

## Hardwareprofile und Sprachfunktionen

Die Konfigurationen unter `config/nox/` bzw. `config/nira/` müssen der tatsächlichen
Hardware entsprechen. Niras neues BNO055-Profil nimmt wie Nox Bus 1, Adresse 0x28
und Einbaulage P1 an. Vor Betrieb Adresse und Achsenzuordnung prüfen.
Servo-USB-Port und Niras Lidar-Port müssen unterschiedliche tatsächliche Geräte
bezeichnen. Insbesondere `/dev/ttyUSB0` in
`rumblex_lidar_2d/config/nira/lidar.yaml` ist ein vorhandener Standardwert; bei
mehreren USB-Adaptern einen passenden stabilen `/dev/serial/by-id/`-Pfad eintragen.
Mikrofon und Lautsprecher über `wpctl status` prüfen und bei Bedarf mit
`wpctl set-default <ID>` auswählen.

Die individuellen Modelle `rumblex_communication/models/nox.onnx` und `nira.onnx`
sind durch `.gitignore` ausgeschlossen und werden bei einem frischen Clone nicht
mitgeliefert. Das Setup lädt nur die öffentlichen openWakeWord-Audiomodelle.
Das eigene Robotermodell vor dem Setup an diesen Pfad kopieren oder anschließend
kopieren und das Setup erneut ausführen. Ohne dieses Modell ist die
Wakeword-Erkennung nicht einsatzbereit; der normale Start aktiviert sie nicht.

Für Online-Spracherkennung eine eigene Google-Service-Account-Datei bereitstellen
und `~/.config/rumblex/speech.env` anlegen:

```ini
GOOGLE_APPLICATION_CREDENTIALS=/home/ROBOTUSER/.config/rumblex/google-speech.json
```

Den tatsächlichen absoluten Pfad verwenden, Dateien mit `chmod 600` schützen und
den Dienst neu starten. Zugangsdaten werden vom Setup weder erzeugt noch kopiert.
Weitere Details stehen in [rumblex_communication](../../rumblex_communication/readme.md).

Grundlagen: [ROS-Installationsanleitung](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Installation/Ubuntu-Install-Debs.rst),
[Lyrical-Plattformen](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Releases/lyrical/supported-platforms.rst),
[YDLIDAR-Treiber](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble).
