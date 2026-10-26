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
ROS-Treiber aus dem `lyrical`-Branch des SteinRobotics-Forks unter
`src/ydlidar_ros2_driver`. Der SDK-Checkout bleibt bei Wiederholung erhalten.
Für den ROS-Treiber setzt das Setup `origin` auf
`git@github.com:SteinRobotics/ydlidar_ros2_driver.git`, wechselt auf `lyrical`
und übernimmt veröffentlichte Änderungen ausschließlich per Fast-forward.
Lokale Änderungen oder unveröffentlichte Commits auf `lyrical` führen zum Abbruch;
sie werden nicht überschrieben. Andere lokale Branches bleiben erhalten.
Der ausführende Benutzer auf dem Pi benötigt SSH-Zugriff auf GitHub.
Die Lyrical-Anpassungen sind bereits im Fork enthalten; ein zusätzlicher Patch
wird nicht mehr angewendet.

Ein vorhandener Checkout aus dem früheren Setup enthält möglicherweise dessen
lokalen CMake-Patch. Änderungen zuerst prüfen und bei Bedarf sichern:

```bash
git -C ~/Workspace/colcon_rumblex/src/ydlidar_ros2_driver diff
git -C ~/Workspace/colcon_rumblex/src/ydlidar_ros2_driver stash push -u -m before-lyrical-migration
./scripts/update_ydlidar_driver.sh
```

Den alten Kompatibilitätspatch nicht auf `lyrical` zurückspielen. Eigene Änderungen
gezielt übernehmen. Anschließend das Setup erneut mit `nira` ausführen.
Der Aktualisierungshelfer verändert keine Build-Artefakte; ein neuer Build ist nötig.

Bestehende Build-Verzeichnisse müssen zu diesem Pi, Ubuntu und ROS
Lyrical passen; keine Build-/Install-Artefakte vom Entwicklungsrechner kopieren.

Das Setup aktiviert I²C, installiert udev-Regeln für GPIO/I²C/USB-Serial/Joystick,
vergibt Hardwaregruppen und erlaubt dem HMI ausschließlich
`sudo shutdown -h now` ohne Passwort. Die Firmwaredatei wird vor der ersten
Ergänzung als `config.txt.before-rumblex` gesichert. WLAN wird nicht verändert.

## WLAN auf 2,4 GHz begrenzen

Nox verwendet Netplan mit `systemd-networkd` und `wpa_supplicant` auf `wlan0`.
NetworkManager/`nmcli` ist dafür nicht erforderlich. Das Setup-Skript setzt selbst
keine Bandbegrenzung; eine Neuinstallation muss sie separat erhalten.

Bei der Prüfung am 26.10.2026 war Nox bereits auf 2,4 GHz begrenzt: Das einzige
aktive WLAN-Profil `FaSt7` (Netzwerk-ID `0`) enthielt ausschließlich Frequenzen
zwischen 2412 und 2484 MHz in `freq_list`. Die Verbindung lief auf 2437 MHz
(Kanal 6). Es wurde keine Netzwerkkonfiguration geändert. Die rootgeschützte
Netplan-Datei konnte ohne interaktive sudo-Anmeldung nicht zusätzlich geprüft
werden; dieser Befund bestätigt die aktive Begrenzung, nicht separat deren
Persistenz nach einem Neustart.

Die aktuelle Verbindung und die erlaubten Frequenzen getrennt prüfen:

```bash
ssh nox@nox
wpa_cli -i wlan0 status
wpa_cli -i wlan0 list_networks
wpa_cli -i wlan0 get_network 0 freq_list
```

Die Netzwerk-ID aus `list_networks` übernehmen und bei mehreren Profilen jedes
prüfen. `freq=2437` allein belegt nur die aktuelle Verbindung. Erst eine
`freq_list` ausschließlich im 2,4-GHz-Bereich belegt die aktive Bandbegrenzung.
Eine leere Antwort oder `FAIL` bestätigt keine Begrenzung.

Die persistente Einstellung lässt sich ohne Ausgabe des WLAN-Passworts prüfen:

```bash
sudo netplan get network.wifis.wlan0.access-points.FaSt7.band
```

Erwartet wird `2.4GHz`. Falls die Begrenzung fehlt, für das bereits konfigurierte
WLAN eine separate Netplan-Datei ergänzen (`FaSt7` durch die eigene SSID ersetzen;
bei mehreren SSIDs jeweils wiederholen):

```bash
sudo netplan set --origin-hint 90-rumblex-wifi-band \
  'network.wifis.wlan0.access-points.FaSt7.band=2.4GHz'
sudo netplan generate
sudo netplan try --timeout 120
```

Netplan legt die Ergänzung unter `/etc/netplan/90-rumblex-wifi-band.yaml` ab;
die vorhandene SSID-, Passwort- und DHCP-Konfiguration bleibt in ihrer bisherigen
Datei. Einen festen Kanal nicht vorgeben. Während `netplan try` in einem zweiten
Terminal erneut per SSH verbinden und die Frequenzbegrenzung prüfen, dann im
ersten Terminal bestätigen. Ohne Bestätigung wird die laufende Konfiguration
zurückgesetzt; bei einem Abbruch auch die Dateien auf der Platte prüfen, bevor
der Pi neu gestartet wird. Diese Systemänderung benötigt das sudo-Passwort des
Roboterbenutzers; die HMI-Freigabe für Shutdown/Reboot genügt dafür nicht.

Referenz: [Netplan-WLAN-Einstellungen](https://netplan.readthedocs.io/en/stable/netplan-yaml/#properties-for-device-type-wifis),
[`netplan set`](https://netplan.readthedocs.io/en/stable/netplan-set/) und
[`netplan try`](https://netplan.readthedocs.io/en/stable/netplan-try/).

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

### Herunterfahren, Neustart und sudo-Rechte

Die Servicebefehle oben als Roboterbenutzer ausführen (auf Nox: `ssh nox@nox`).
Der Benutzer verwaltet seinen Dienst unter `~/.config/systemd/user/` ohne sudo.
`--user` ist erforderlich: Ohne diese Option wird der systemweite Dienst gesucht.

Herunterfahren und Neustarten betreffen den gesamten Pi und benötigen über SSH
Administratorrechte. `NOPASSWD` erlaubt einen Aufruf **mit sudo ohne Passwort**;
es erlaubt nicht automatisch den Aufruf ohne sudo. `/sbin/restart` existiert auf
Nox nicht; für einen Neustart des Pi `sudo reboot` verwenden.
`systemctl --user restart autostart_ros2.service` startet nur den Roboterdienst neu.

Das Setup installiert unter `/etc/sudoers.d/rumblex-<benutzer>` ausschließlich die
Freigabe für `sudo shutdown -h now`, die das Hardware-HMI benötigt. `sudo reboot`
verlangt weiterhin ein Passwort, sofern es nicht zusätzlich freigegeben wurde.
Eine Regel für den Benutzer `rumblex` gilt nicht für `nox`.

Fehlt die Freigabe auf einer bestehenden Nox-Installation, die Datei mit visudo
anlegen bzw. bearbeiten:

```bash
sudo visudo -f /etc/sudoers.d/rumblex-nox
```

Für das HMI reicht diese Regel:

```sudoers
nox ALL=(root) NOPASSWD: /usr/sbin/shutdown -h now
```

Für einen zusätzlich erlaubten Neustart ohne Passwort stattdessen:

```sudoers
nox ALL=(root) NOPASSWD: /usr/sbin/shutdown -h now, /usr/sbin/reboot
```

Bei anderen Login-Benutzern `nox` und den Dateinamen entsprechend ersetzen.
Ein erneuter Setup-Lauf ersetzt diese Datei wieder durch die reine HMI-Freigabe.
Auf Nox verweist `/sbin` auf `/usr/sbin`; die Regeln verwenden den vollständigen
Pfad unter `/usr/sbin`. Keine sudoers-Freigabe für `systemctl` ist zur Verwaltung
des Benutzerdienstes nötig.

Die Berechtigungen lassen sich mit `sudo -l` prüfen, ohne den Pi herunterzufahren.
Nach Einrichtung der passenden Freigabe lauten die tatsächlichen Aufrufe
`sudo shutdown -h now` bzw. `sudo reboot`.

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
[YDLIDAR-Treiber](https://github.com/SteinRobotics/ydlidar_ros2_driver/tree/lyrical).
