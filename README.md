# BLVD20KM_asukiaaa

A library for Arduino to control BLVD20KM which is a brushless motor controller of Oriental Motor.

# Usage

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
