# Nox hardware bill of materials

Quantities are per robot and exclude spares. **Confirmed** means confirmed by the owner;
**Documented** means identified in repository configuration or hardware documentation;
**CAD** means present in the mechanical design, with installation still to be confirmed.
**TBD** marks missing specifications or quantities. Mechanical assemblies below exclude the
electronics listed separately.

## Compute and motion

| Qty | Component | Specification | Status / notes |
| --- | --- | --- | --- |
| 1 | Main computer | Raspberry Pi 5, 8 GB RAM | Confirmed |
| TBD | Boot storage | Medium, capacity, and model TBD | Confirm installed storage |
| TBD | Pi cooling | Heatsink / fan model TBD | Confirm installed cooling |
| 18 | Leg servos | Hiwonder HX-35H | Model confirmed; six legs with three servos each in [servo configuration](rumblex_movement/config/nox/servo_description.yaml) |
| 2 | Head servos | Hiwonder HX-35H | Model confirmed; yaw and pitch in the same configuration; 20 servos total |
| 1 | USB servo bus interface | Hiwonder-compatible; exact board/model TBD | Documented USB serial connection; [CAD interface board](rumblex_description/cad/boards/board_servo_interface.py) |
| 1 | Servo distribution board | Ten 5264-3P connectors | [CAD](rumblex_description/cad/boards/board_servo_plug.py); confirm installed board/revision; connectors included in this assembly |

## Power

| Qty | Component | Specification | Status / notes |
| --- | --- | --- | --- |
| 2 | Battery packs | OVONIC, 2200 mAh, 35C | Brand, capacity, rating, and count confirmed; confirm cell count, nominal voltage, connector, and exact SKU |
| TBD | Battery harness | Connector types, wire gauge, lengths, and pack wiring TBD | Confirm whether packs feed separate circuits or share a supply |
| 1 | DC/DC power converter | 52Pi RPi PD Power, 12 V to 5 V | Confirmed model and conversion; output current rating TBD |
| 1 | Servo power relay module | Standard 3.3 V relay module | Type confirmed; [example supplier listing](https://www.amazon.de/dp/B09PNRXW5M), not a confirmed exact model; contact rating TBD. [GPIO16, physical pin 36](README.md#raspberry-pi-5-pin-layout) |
| 1 | Power monitor | INA228 module; exact board/shunt TBD | [Documented](rumblex_hmi/README.md) |
| TBD | Main switch, fuse, and power connectors | Types, ratings, and quantities TBD | Confirm installed parts |

## Sensors and user interface

| Qty | Component | Specification | Status / notes |
| --- | --- | --- | --- |
| 1 | Single-beam lidar | Garmin LIDAR-Lite v3 | [Documented](SENSORS.md); I2C connection |
| 1 | Lidar supply capacitor | 680 µF; voltage rating and part number TBD | Listed in [hardware wiring notes](rumblex_doc/readme.md#garmin-lidar-lite-v3); confirm installed part |
| 1 | IMU | BNO055; CJMCU-055 board referenced in hardware notes | [Documented](rumblex_doc/readme.md#cjmcu-055-bno055); confirm exact board |
| 1 | Display | SSD1306, 128 × 64, I2C | [Documented](rumblex_hmi/README.md); physical size/model TBD |
| 1 | I2C distribution board | Exact board/revision TBD | [CAD](rumblex_description/cad/boards/board_i2c_distributor.py) |
| TBD | Microphone | Model, quantity, and connection TBD | Speech input is documented; exact hardware is missing |
| TBD | Speaker / USB audio assembly | Exact model and quantity TBD | [Hardware notes](rumblex_doc/readme.md#loudspeaker) identify Jieli UACDemoV1.0 USB audio, not the complete speaker product |

## Mechanical parts and wiring

| Qty | Component | Specification | Status / notes |
| --- | --- | --- | --- |
| 1 set | Torso plates, chassis, and covers | Nox-specific custom parts | [CAD sources](rumblex_description/cad/robot_nox); fabrication materials and part revisions TBD |
| 6 sets | Leg structures and feet | Coxa, femur, tibia, brackets, and feet | [CAD sources](rumblex_description/cad/robot_nox); servos counted separately above |
| 1 set | Head and lidar mounting structure | Nox-specific head assembly | [CAD sources](rumblex_description/cad/robot_nox); servos and lidar counted separately above |
| TBD | Servo horns and brackets | HX-35H-compatible parts | Confirm models, counts, and which parts are supplied with the servos |
| TBD | Fasteners and standoffs | Sizes, lengths, materials, and counts TBD | Includes PCB, chassis, and servo mounting hardware |
| TBD | Signal and USB cables | Servo bus, I2C, and USB harnesses | Connector types, lengths, and counts TBD; 5264-3P referenced in [hardware notes](rumblex_doc/readme.md#wire) |

## External equipment

| Qty | Component | Specification | Status / notes |
| --- | --- | --- | --- |
| TBD | Teleoperation controller | PS3-compatible controller referenced in [hardware notes](rumblex_doc/readme.md#ps3-controller) | Confirm exact model and whether shared between robots |
| TBD | Battery charger and charging leads | Model and battery connector compatibility TBD | Confirm dedicated or shared equipment |

## Information still needed

- Battery cell count/voltage, connector/SKU, and electrical arrangement of the two packs.
- Storage, cooling, converter current rating, servo interface, relay contact rating, and protection hardware.
- Exact sensor breakout/display models, microphone, speaker, and amplifier/audio interface.
- Mechanical materials/revisions, fastener counts, cable lengths, and included servo accessories.
- Controller and charger models and whether they are shared with Nira.

The [IMU configuration](rumblex_bringup/config/nox/bno055_params.yaml) uses address `0x28`,
while the root README lists `0x29`; the installed address remains to be confirmed.
