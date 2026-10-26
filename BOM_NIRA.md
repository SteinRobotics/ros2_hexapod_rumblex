# Nira hardware bill of materials

Quantities are per robot and exclude spares. **Confirmed** means confirmed by the owner;
**Documented** means identified in repository configuration or hardware documentation;
**CAD** means present in the mechanical design, with installation still to be confirmed.
**TBD** marks missing specifications or quantities. Nira is under development; configured and
modelled components do not establish that the hardware has been installed or commissioned.
Mechanical assemblies below exclude the electronics listed separately.

## Compute and motion

| Qty | Component                | Specification                             | Status / notes                                                                                                                                             |
| --- | ------------------------ | ----------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | Main computer            | Raspberry Pi 5, 8 GB RAM                  | Confirmed                                                                                                                                                  |
| TBD | Boot storage             | Medium, capacity, and model TBD           | Confirm selected storage                                                                                                                                   |
| TBD | Pi cooling               | Heatsink / fan model TBD                  | Confirm selected cooling                                                                                                                                   |
| 18  | Leg servos               | Feetech HLS3950M                          | Model confirmed; six legs with three servos each in [servo configuration](rumblex_movement/config/nira/servo_description.yaml)                             |
| 2   | Head servos              | Feetech HLS3950M                          | Model confirmed; yaw and pitch in the same configuration; 20 servos total                                                                                  |
| 1   | USB servo bus interface  | Feetech-compatible; exact board/model TBD | Documented USB serial connection; [CAD board layout](rumblex_description/cad/robot_nira/board_layout.py)                                                   |
| 1   | Servo distribution board | Ten 5264-3P connectors                    | [CAD](rumblex_description/cad/boards/board_servo_plug.py); confirm board/revision and actual connector compatibility; connectors included in this assembly |

## Power

| Qty | Component                               | Specification                                             | Status / notes                                                                                                                   |
| --- | --------------------------------------- | --------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| 2   | Battery packs                           | OVONIC, 2200 mAh, 35C                                     | Brand, capacity, rating, and count confirmed; confirm cell count, nominal voltage, connector, and exact SKU                      |
| TBD | Battery harness                         | Connector types, wire gauge, lengths, and pack wiring TBD | Confirm whether packs feed separate circuits or share a supply                                                                   |
| 1   | DC/DC power converter                   | 52Pi RPi PD Power, 12 V to 5 V                            | Confirmed model and conversion; output current rating TBD                                                                        |
| 1   | Servo power relay module                | Standard 3.3 V relay module                               | Type confirmed; [example supplier listing](https://www.amazon.de/dp/B09PNRXW5M), not a confirmed exact model; contact rating TBD |
| 1   | Power monitor                           | INA228 module; exact board/shunt TBD                      | CAD board layout and shared HMI implementation; confirm selection                                                                |
| TBD | Main switch, fuse, and power connectors | Types, ratings, and quantities TBD                        | Confirm selected parts                                                                                                           |

The [battery CAD envelope](rumblex_description/cad/common/battery.py) assumes a 3S, 11.1 V
OVONIC short pack with a Deans/T-plug and a nominal 74 × 33 × 26 mm body. Older
[hardware notes](rumblex_doc/readme.md#akku) mention XT60 instead. These details remain
unconfirmed and are not treated as a confirmed battery SKU.

## Sensors and user interface

| Qty | Component                   | Specification                                 | Status / notes                                                                                                                                                                   |
| --- | --------------------------- | --------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | Planar lidar                | YDLIDAR T-mini                                | [Documented](SENSORS.md); confirm exact supplied variant                                                                                                                         |
| 1   | Lidar interface board       | T-mini USB/serial interface                   | [CAD](rumblex_description/cad/robot_nira/board_ydlidar_tmini_interface.py); confirm part number and whether supplied with lidar                                                  |
| 1   | Head camera                 | OBSBOT Meet SE                                | [Documented](rumblex_camera/README.md); USB connection                                                                                                                           |
| 1   | IMU                         | BNO055; exact breakout board TBD              | [Configured](rumblex_bringup/config/nira/bno055_params.yaml) and present in CAD; confirm hardware selection                                                                      |
| TBD | Display                     | SSD1306, 128 × 64, I2C expected by shared HMI | [Software support](rumblex_hmi/README.md); confirm inclusion, quantity, size, and model                                                                                          |
| 1   | I2C distribution board      | Exact board/revision TBD                      | [CAD board layout](rumblex_description/cad/robot_nira/board_layout.py)                                                                                                           |
| TBD | Microphone                  | Model, quantity, and connection TBD           | Confirm whether a separate microphone or camera audio is intended                                                                                                                |
| 2   | Loudspeaker drivers         | Nominal 40 mm front diameter, 22 mm depth     | [CAD placement](rumblex_description/cad/robot_nira/chassis_slope_cover.py) and [envelope](rumblex_description/cad/common/loudspeaker.py); actual model, impedance, and power TBD |
| TBD | Audio interface / amplifier | Model, channel count, and supply TBD          | Confirm hardware driving the speakers                                                                                                                                            |

## Mechanical parts and wiring

| Qty    | Component                                           | Specification                                      | Status / notes                                                                                                  |
| ------ | --------------------------------------------------- | -------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| 1 set  | Torso plates, chassis, covers, and battery supports | Nira-specific custom parts                         | [CAD sources](rumblex_description/cad/robot_nira); fabrication materials and part revisions TBD                 |
| 6 sets | Leg structures and feet                             | Coxa, femur, tibia, brackets, and foot assemblies  | [CAD sources](rumblex_description/cad/robot_nira); servos counted separately above                              |
| 1 set  | Camera head structure                               | Camera cage and head brackets                      | [Head CAD](rumblex_description/cad/robot_nira/assembly_head.py); camera and servos counted separately above     |
| 1 set  | Lidar mounting structure and interface housing      | Nira-specific custom parts                         | [Torso CAD](rumblex_description/cad/robot_nira/assembly_torso.py); lidar and interface counted separately above |
| TBD    | Servo horns and brackets                            | HLS3950M-compatible parts                          | Confirm models, counts, and which parts are supplied with the servos                                            |
| TBD    | Fasteners and standoffs                             | Sizes, lengths, materials, and counts TBD          | Includes PCB, chassis, servo, camera, and lidar mounting hardware                                               |
| TBD    | Signal and USB cables                               | Servo bus, I2C, lidar, camera, and audio harnesses | Connector types, lengths, and counts TBD                                                                        |

## External equipment

| Qty | Component                          | Specification                                 | Status / notes                                |
| --- | ---------------------------------- | --------------------------------------------- | --------------------------------------------- |
| TBD | Teleoperation controller           | Exact model TBD                               | Confirm selection and whether shared with Nox |
| TBD | Battery charger and charging leads | Model and battery connector compatibility TBD | Confirm dedicated or shared equipment         |

## Information still needed

- Battery cell count/voltage, connector/SKU, and electrical arrangement of the two packs.
- Storage, cooling, converter current rating, servo interface, relay contact rating, and protection hardware.
- Confirmation of BNO055, INA228, SSD1306, and I2C distribution hardware, including exact modules.
- Lidar kit contents, microphone source, speaker model, and audio interface/amplifier.
