#include "BLVD20KM_asukiaaa.h"

#define FN_CODE_READ        0x03
#define FN_CODE_WRITE       0x06
#define FN_CODE_DIAGNOSIS   0x08
#define FN_CODE_WRITE_MULTI 0x10

#define BAUDRATE 115200 // depends on SW2:[1-3]

#define ADDR_SPEED0_H      0x0480
#define ADDR_SPEED0_L      0x0481
#define ADDR_MOTOR_CONTROL 0x007d
#define ADDR_ANALOG_MODE_L 0x10e3
#define ADDR_CONFIG_H      0x018c
#define ADDR_CONFIG_L      0x018d
#define MOTOR_DIRECTOIN_STOP    0
#define MOTOR_DIRECTOIN_FORWARD 1
#define MOTOR_DIRECTOIN_REVERSE 2

#define MOTOR_FORWARD_BIT      B00001000
#define MOTOR_REVERSE_BIT      B00010000
#define MOTOR_SLOW_CHANGE_BIT  B00100000
#define MOTOR_FREE_ON_STOP_BIT B10000000

// #define DEBUG_PRINT

unsigned short getCRC16(unsigned char const *buf, unsigned short len) {
  // Serial.print("data len: ");
  // Serial.println(len);
  // for (unsigned short i = 0; i < len; ++i) {
  //   Serial.print(ptr[(unsigned char) i], HEX);
  //   Serial.print(" ");
  //   if (i > 30) break;
  // }
  // Serial.println("");

  unsigned short crc = 0xFFFF;
  for (unsigned short pos = 0; pos < len; pos++) {
    crc ^= (unsigned short)buf[pos];    // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }

  return crc;
}

BLVD20KM_asukiaaa::BLVD20KM_asukiaaa(HardwareSerial* serial, unsigned char address, unsigned char dePin, unsigned char rePin) {
  this->serial = serial;
  this->address = address;
  this->dePin = dePin;
  this->rePin = rePin;
}

// #ifndef __arm__
// BLVD20KM_asukiaaa::BLVD20KM_asukiaaa(SoftwareSerial* serial, unsigned char address, unsigned char dePin, unsigned char rePin) {
//   this->softserial = serial;
//   this->address = address;
//   this->dePin = dePin;
//   this->rePin = rePin;
// }
// #endif

void BLVD20KM_asukiaaa::begin() {
  serial->begin(BAUDRATE, SERIAL_8E1);
  pinMode(dePin, OUTPUT);
  pinMode(rePin, OUTPUT);
  digitalWrite(dePin, LOW);
  digitalWrite(rePin, LOW);
  writeSpeedControlMode(BLVD02KM_SPEED_MODE_USE_DIGITALS);
  writeSpeed(BLVD20KM_SPEED_MIN);
  writeStop();
  // crcTest(); // for debugging
}

unsigned char BLVD20KM_asukiaaa::writeSpeedControlMode(unsigned short mode) {
  unsigned char result;
  result = writeRegister(ADDR_ANALOG_MODE_L, mode);
  if (result != 0) {
    return result;
  }
  return writeConfigTrigger(); // trigger after setting ADDR_ANALOG_MODE
}

unsigned char BLVD20KM_asukiaaa::readSpeedControlMode(unsigned short *mode) {
  return readRegisters(ADDR_ANALOG_MODE_L, 1, mode);
}

unsigned char BLVD20KM_asukiaaa::writeConfigTrigger() {
  return writeRegister(ADDR_CONFIG_L, 1);
}

unsigned short createMotorControl16bit(unsigned char motorDirection, bool freeLockOnStop = true, bool slowChange = true, unsigned char motorDataNum = 0) {
  // MB-FREE, -, STOP-MODE, REV, FWD, M1, M2, M0
  unsigned short bits = 0x0000;
  switch (motorDirection) {
  case MOTOR_DIRECTOIN_REVERSE:
    bits |= MOTOR_REVERSE_BIT;
    break;
  case MOTOR_DIRECTOIN_FORWARD:
    bits |= MOTOR_FORWARD_BIT;
    break;
  }
  if (freeLockOnStop) {
    bits |= MOTOR_FREE_ON_STOP_BIT;
  }
  if (slowChange) {
    bits |= MOTOR_SLOW_CHANGE_BIT;
  }
  if (motorDataNum != 0 && motorDataNum < B1000) {
    bits |= motorDataNum;
  }
  return bits;
}

unsigned char BLVD20KM_asukiaaa::writeForward() {
#ifdef DEBUG_PRINT
  Serial.println("forward");
#endif
  // unsigned short data16bit = B10101000; // forward and unlock blake
  // unsigned short data16bit = B10001000;
  // return writeRegister(ADDR_MOTOR_CONTROL, data16bit);
  return writeRegister(ADDR_MOTOR_CONTROL, createMotorControl16bit(MOTOR_DIRECTOIN_FORWARD));
}

unsigned char BLVD20KM_asukiaaa::writeLock() {
#ifdef DEBUG_PRINT
  Serial.println("lock");
#endif
  return writeRegister(ADDR_MOTOR_CONTROL, createMotorControl16bit(MOTOR_DIRECTOIN_STOP, false));
}

unsigned char BLVD20KM_asukiaaa::writeStop() {
#ifdef DEBUG_PRINT
  Serial.println("stop");
#endif
  return writeRegister(ADDR_MOTOR_CONTROL, createMotorControl16bit(MOTOR_DIRECTOIN_STOP));
}

unsigned char BLVD20KM_asukiaaa::writeReverse() {
#ifdef DEBUG_PRINT
  Serial.println("reverse");
#endif
  return writeRegister(ADDR_MOTOR_CONTROL, createMotorControl16bit(MOTOR_DIRECTOIN_REVERSE));
}

unsigned char BLVD20KM_asukiaaa::writeSpeed(unsigned short speed) {
#ifdef DEBUG_PRINT
  Serial.println("setSpeed " + String(speed));
#endif
  return writeRegister(ADDR_SPEED0_L, speed);
}

unsigned char BLVD20KM_asukiaaa::writeDiagnosis() {
  unsigned char result;
  unsigned char data[41];
  data[0] = 0;
  data[1] = 0;
  data[2] = 1;
  data[3] = 2;
  writeQuery(FN_CODE_DIAGNOSIS, data, sizeof(data));
  for (unsigned char i = 0; i < 41; ++i) {
    data[i] = 0;
  }
  result = readQuery(FN_CODE_DIAGNOSIS, data, sizeof(data));
  if (result != 0) { return result; }
  if (data[2] != 1 || data[3] != 2) {
    return BLVD20KM_ERROR_DIAGNOSIS_DATA_INVALID;
  }

  return 0;
}

unsigned char BLVD20KM_asukiaaa::readDirection(boolean *forwarding, boolean *reversing, boolean *freeLockOnStop) {
  unsigned short data;
  unsigned char result;
  result = readRegisters(ADDR_MOTOR_CONTROL, 1, &data);
  if (result != 0) {
    return result;
  }
  *forwarding = (MOTOR_FORWARD_BIT & data) != 0x00;
  *reversing = (MOTOR_REVERSE_BIT & data) != 0x00;
  *freeLockOnStop = (MOTOR_FREE_ON_STOP_BIT & data) != 0x00;
  return 0;
}

unsigned short uCharsToUShort(unsigned char *chars) {
  return ((unsigned short) chars[0]) << 8 | (unsigned short) chars[1];
}

unsigned long uShortsToULong(unsigned short *shorts) {
  return ((unsigned long) shorts[0]) << 16 | (unsigned long) shorts[1];
}

unsigned char BLVD20KM_asukiaaa::readSpeed(unsigned short *speed) {
  static const unsigned short dataLen = 2;
  static unsigned short data[dataLen];
  unsigned char result;
  result = readRegisters(ADDR_SPEED0_H, dataLen, data);
  if (result != 0) {
    return result;
  }
  *speed = uShortsToULong(data);
  return 0;
}

unsigned char BLVD20KM_asukiaaa::writeRegister(unsigned short writeAddress, unsigned short data16bit) {
  unsigned char data[] = {
    highByte(writeAddress),
    lowByte(writeAddress),
    highByte(data16bit),
    lowByte(data16bit)
  };
  writeQuery(FN_CODE_WRITE, data, sizeof(data));
  return readQuery(FN_CODE_WRITE, data, sizeof(data));
}

unsigned char BLVD20KM_asukiaaa::readRegisters(unsigned short readStartAddress, unsigned short dataLen, unsigned short* registerData) {
  unsigned char result;
  unsigned char data[] = {
    highByte(readStartAddress),
    lowByte(readStartAddress),
    highByte(dataLen),
    lowByte(dataLen)
  };
  writeQuery(FN_CODE_READ, data, sizeof(data));
  static unsigned char rData[41];
  result = readQuery(FN_CODE_READ, rData, dataLen * 2 + 1);
  if (result != 0) { return result; }
  // Serial.println("");
  // Serial.println(rDataLen);
  // Serial.println(data16bitLen * 2 + 1);
  for (unsigned short i = 0; i < dataLen; ++i) {
    registerData[i] = uCharsToUShort(&rData[i * 2 + 1]); // + 1 to skip data length byte
  }
  return 0;
}

void BLVD20KM_asukiaaa::writeQuery(unsigned char fnCode, unsigned char* data, unsigned int dataLen) {
  digitalWrite(dePin, HIGH);
  digitalWrite(rePin, HIGH);
  delay(10);
  unsigned int queryLen = 4 + dataLen;
  unsigned int i;
  queryBuffer[0] = address;
  queryBuffer[1] = fnCode;
  for (i = 0; i < dataLen; ++i) {
    queryBuffer[i+2] = data[i];
  }

  // debug for address 3 device
  // queryBuffer[0] = 0x03;
  // queryBuffer[1] = 0x08;
  // queryBuffer[2] = 0x00;
  // queryBuffer[3] = 0x00;
  // queryBuffer[4] = 0x12;
  // queryBuffer[5] = 0x34;

  unsigned short crc16 = getCRC16(queryBuffer, queryLen - 2);
  // Serial.print("crc16: ");
  // Serial.println(crc16, HEX);
  queryBuffer[queryLen - 2] = lowByte(crc16);
  queryBuffer[queryLen - 1] = highByte(crc16);

  // debug for address 3 device
  // queryBuffer[0] = 0x03;
  // queryBuffer[1] = 0x08;
  // queryBuffer[2] = 0x00;
  // queryBuffer[3] = 0x00;
  // queryBuffer[4] = 0x12;
  // queryBuffer[5] = 0x34;
  // queryBuffer[6] = 0xEC;
  // queryBuffer[7] = 0x9E;
  // queryLen = 8;

  while (serial->available()) { serial->read(); } // remove received buffer before sending
#ifdef DEBUG_PRINT
  Serial.print("Send: ");
#endif
  for (i = 0; i < queryLen; ++i) {
    serial->write(queryBuffer[i]);
#ifdef DEBUG_PRINT
    Serial.print(queryBuffer[i], HEX);
    Serial.print(" ");
#endif
  }
  serial->flush();
#ifdef DEBUG_PRINT
  Serial.println("");
#endif
  delayMicroseconds(500);
  digitalWrite(dePin, LOW);
  digitalWrite(rePin, LOW);
  delay(10);
}

unsigned char BLVD20KM_asukiaaa::readQuery(unsigned char fnCode, unsigned char* data, unsigned short dataLen) {
  unsigned short queryLen = 0;
  unsigned long waitFrom = millis();
  const unsigned long timeoutMs = 20;
#ifdef DEBUG_PRINT
  Serial.print("Receive: ");
#endif
  while (serial->available() || millis() - waitFrom < timeoutMs) {
    if (!serial->available()) {
      delay(2);
      continue;
    }
    waitFrom = millis();
    if (queryLen == BLVD20KM_QUERY_MAX_LEN) {
#ifdef DEBUG_PRINT
      Serial.println("stop receiving because buffer length was over");
#endif
      return BLVD20KM_ERROR_OVER_QUERY_MAX_LEN;
    }
    queryBuffer[queryLen] = serial->read();
#ifdef DEBUG_PRINT
    Serial.print(queryBuffer[queryLen], HEX);
    Serial.print(" ");
#endif
    ++queryLen;
  }
#ifdef DEBUG_PRINT
  Serial.println("");
#endif
  if (queryLen == 0) {
    return BLVD20KM_ERROR_NO_RESPONSE;
  }

  unsigned short crc = getCRC16(queryBuffer, queryLen - 2);
  if (highByte(crc) != queryBuffer[queryLen - 1] || lowByte(crc) != queryBuffer[queryLen - 2]) {
    return BLVD20KM_ERROR_UNMATCH_CRC;
  }
  if (queryBuffer[0] != address) {
    return BLVD20KM_ERROR_UNMATCH_ADDRESS;
  }
  if (queryBuffer[1] != fnCode) {
    if (queryBuffer[1] == (fnCode + 0x80)) {
      return queryBuffer[2]; // ERROR_CODE
    } else {
      return BLVD20KM_ERROR_UNMATCH_FN_CODE;
    }
  }

  if (dataLen != queryLen - 4) {
    return BLVD20KM_ERROR_UNMATCH_DATA_LEN;
  }

  for (unsigned short i = 0; i < dataLen; ++i) {
    data[i] = queryBuffer[i + 2];
  }
  return 0;
}
