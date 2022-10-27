# BLVD20KM_asukiaaa

A library for Arduino to control BLVD20KM or BLVD40NM which is a brushless motor driver of Oriental Motor.

# Usage

## driver configuration

### SW 2

To use motor driver in 115200bps via RS485 modbus, set switch 2 like this

No | State | Role
-- | ----- | ---
1  | OFF   | Baudrate
2  | OFF   | Baudrate
3  | ON    | Baudrate
4  | OFF   | Not used
5  | ON    | Use Modbus Protocol
6  | OFF   | Not used
7  | ON if the motor driver is end of RS485 chain | End point register
8  | OFF   | Address upper bit

Baudrate configuration

No1 | No2 | No3 | Baudrate
--- | --- | --- | --------
OFF | OFF | OFF | 9600
ON  | OFF | OFF | 19200
OFF | ON  | OFF | 38400
ON  | ON  | OFF | 57600
OFF | OFF | ON  | 115200

### SW 3

Switch 3 means address of motor driver.
If you use multiple motor, set different address for each motors.

Example project communicate with a driver which address is 1.

## Connection

### LAN cable pinout of motor driver

No | Role
---|-----
1  | NC
2  | GND
3  | TR+
4  | NC
5  | NC
6  | TR-
7  | NC
8  | NC

### Arduino <-> Uart to RS485 Transceiver <-> motor driver

Arduino | Uart to RS485 | motor driver
--------|--------|------
None    | A      | TR+ (LAN 3)
None    | B      | TR- (LAN 6)
TX1     | DI     | None
RX1     | RO     | None
D4      | RE     | None
D5      | DE     | None
5V      | 3~6V   | None
GND     | GND    | GND (LAN 2)

I checked connection with using some RS485 tranceiver. Ex: [SparkFun Transceiver Breakout - RS-485](https://www.sparkfun.com/products/10124), [sp3485-breakout](https://www.switch-science.com/catalog/6822/) or [sp3485-one-line-breakout](https://www.switch-science.com/catalog/6823/).

## Software

See [example project](./examples) and [header file](./src/BLVD20KM_asukiaaa.h).

## Alarms

Alarm information from ユーザーマニュアル通信編 HM-5101-5J.pdf 35-36 pages and BLV Series USER MANUAL (RS-485 Communication Mode) HM-5114E.pdf 36-37 pages.

Code HEX (Dec) | Role in Japanese | Role in English
-------------- | ---- | ----
0x20 (32) | 過電流 | Overcurrent
0x21 (33) | 主回路加熱 | Main ciruit overheat
0x22 (34) | 過電圧 | Overvoltage
0x25 (37) | 不足電圧 | Undervoltage
0x28 (40) | センサ異常 | Sensor error
0x2D (45) | 主回路出力異常 | Main circuit output error
0x30 (48) | 過負荷 | Overload
0x31 (49) | 過速度 | Overspeed
0x41 (65) | EEPROM異常 | EEPROM error
0x42 (66) | 初期時センサ異常 | Initial sensor error
0x46 (70) | 初期時運転禁止 | Prevention of operation at power on
0x6e (110) | 外部停止 | External stop
0x81 (129) | ネットワークバス異常 | Network bus error
0x83 (131) | 通信用スイッチ設定異常 | Communicatino switch setting error
0x84 (132) | RS485 通信異常 | RS485 communication error
0x85 (133) | RS485タイムアウト | RS485 communication timeout
0x8E (142) | ネットワークコンバーター異常 | Network converter error

# License

MIT

# References

- BLVD20KM manual list [ja](https://www.orientalmotor.co.jp/download/manual_search.action?productName=BLVD20KM&searchPattern=1&gengoId=1) [en](https://www.orientalmotor.co.jp/download/manual_search.action?productName=BLVD20KM&searchPattern=1&gengoId=2&x=22&y=17)
- [BLVD20KM (24V motor driver)](https://www.orientalmotor.co.jp/products/detail.action?hinmei=BLVD20KM)
- [BLVM62* (24V motors)](https://www.orientalmotor.co.jp/product_search/result.action?productName=BLVM62&searchPattern=1)
- [BLVD40NM (48V motor driver)](https://www.orientalmotor.co.jp/products/detail.action?hinmei=BLVD40NM)
- [BLVM64* (48V motors)](https://www.orientalmotor.co.jp/product_search/result.action?productName=BLVM64&searchPattern=1)
- [ArduinoでRS-485に対応する](https://www.denshi.club/cookbook/wire/rs-4852-arduinors-485.html)
- [CRC16を計算する](http://www.soramimi.jp/crc16/)
- [SP3485 datasheet (PDF)](https://media.digikey.com/pdf/Data%20Sheets/MaxLinear%20PDFs/SP3481,85.pdf)
