# Hiwonder velocity interface

`CHiwonderProtocol` implements the 115200-baud `55 55` protocol described in
[Hiwonder's Bus Servo Communication Protocol (manufacturer document mirrored by Purdue)](https://engineering.purdue.edu/477grp4/Team/journal/img%20-%20Juho/week7/servo%20bus%20protocol.pdf),
commands 21–22 and response 11 (PDF pages 8–9 and 13).

- `bool setVelocity(uint8_t id, int16_t velocity)` enters motor mode and clamps
  speed to `[-1000, 1000]`. Negative reverses direction; zero stops rotation.
  Units are native speed values; the specification gives no conversion to rad/s.
- `bool getVelocity(uint8_t id, int16_t& velocity)` reads configured motor speed,
  not measured velocity. It returns false in position mode, for invalid/broadcast
  IDs, or on read failure, preserving the output argument.
- `setMotorMode` and `getMotorSpeed` provide the same motor-speed operations.

Wire layout (hexadecimal):

| Operation | Packet |
| --- | --- |
| Set | `55 55 ID 07 1D 01 00 SPEED_LO SPEED_HI CHECKSUM` |
| Read request | `55 55 ID 03 1E CHECKSUM` |
| Read response | `55 55 ID 07 1E MODE 00 SPEED_LO SPEED_HI CHECKSUM` |

Speed is signed 16-bit, little-endian. Checksum is the inverted low byte of the
sum from ID through the last parameter. Writes report transmission success,
without acknowledgement. Mode/speed are not retained across power loss.

There are no acceleration commands in this specification. Position-mode speed
is controlled through the existing target/duration interface. Newer magnetic
encoder models use a [different protocol](https://github.com/Hiwonder-official/hiwonder-servo-sdk/blob/main/docs/protocol_en.md).
