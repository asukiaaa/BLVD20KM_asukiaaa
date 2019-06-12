# BLVD20KM_asukiaaa

A library for Arduino to control BLVD20KM which is a brushless motor controller of Oriental Motor.

# Usage

## BLVD20KM configuration

### SW 2

Change switch 2 like this

No | State | Role
-- | ----- | ---
1  | OFF   | Baudrate
2  | OFF   | Baudrate
3  | ON    | Baudrate
4  | OFF   | Not used
5  | ON    | Use Modbus Protocol
6  | OFF   | Not used
7  | ON if the BLVD02KM is end of RS485 chain | End point register
8  | OFF   | Address upper bit

Set 1-3 as 001 to use 115200bps.

### SW 3

Switch 3 means address of motor driver.
If you use multiple motor, set different address for each motors.

## Connection

### LAN cable pinout of BLVD20KM

1: NC
2: GND
3: TR+
4: NC
5: NC
6: TR-
7: NC
8: NC

### Arduino <-> SP3485 <-> BLVD20KM

Arduino | SP3485 | BLVD20KM
--------|--------|------
None    | A      | TR+ (LAN 3)
None    | B      | TR- (LAN 6)
TX1     | DI     | None
RX1     | RO     | None
D3      | RE     | None
D2      | DE     | None
VCC     | 5V     | None
GND     | GND    | GND (LAN 2)

## Software

See [example project](./examples).

# License

MIT

# References

- [BLVD20KM](https://www.orientalmotor.co.jp/products/detail.action?hinmei=BLVD20KM)
- [ArduinoでRS-485に対応する](https://www.denshi.club/cookbook/wire/rs-4852-arduinors-485.html)
