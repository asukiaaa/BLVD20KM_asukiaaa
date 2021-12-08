#include "BLVD20KM_asukiaaa.h"

#define FN_CODE_DIAGNOSIS 0x08

#define ADDR_ALARM_H 0x0080
#define ADDR_ALARM_L 0x0081
#define ADDR_COMMAND_SPEED_H 0x00c8
#define ADDR_COMMAND_SPEED_L 0x00c9
#define ADDR_FEEDBACK_SPEED_H 0x00ce
#define ADDR_FEEDBACK_SPEED_L 0x00cf
#define ADDR_LOAD_FACTOR_H 0x0108
#define ADDR_LOAD_FACTOR_L 0x0109
#define ADDR_RESET_ALARM_H 0x0180
#define ADDR_RESET_ALARM_L 0x0181
#define ADDR_SPEED0_H 0x0480
#define ADDR_SPEED0_L 0x0481
#define ADDR_TORQUE_H 0x0700
#define ADDR_TORQUE_L 0x0701
#define ADDR_TORQUE_LIMIT0_H 0x0700
#define ADDR_TORQUE_LIMIT0_L 0x0701
#define ADDR_MOTOR_CONTROL 0x007d
#define ADDR_ANALOG_MODE_L 0x10e3
#define ADDR_CONFIG_H 0x018c
#define ADDR_CONFIG_L 0x018d
#define MOTOR_DIRECTOIN_STOP 0
#define MOTOR_DIRECTOIN_FORWARD 1
#define MOTOR_DIRECTOIN_REVERSE 2

#define MOTOR_FORWARD_BIT B00001000
#define MOTOR_REVERSE_BIT B00010000
#define MOTOR_SLOW_CHANGE_BIT B00100000
#define MOTOR_FREE_ON_STOP_BIT B10000000

// #define DEBUG_PRINT

BLVD20KM_asukiaaa::BLVD20KM_asukiaaa(HardwareSerial *serial, uint8_t address,
                                     uint8_t dePin, uint8_t rePin)
    : createdModbus(true) {
  this->address = address;
  modbus = new rs485_asukiaaa::ModbusRtu::Central(serial, dePin, rePin);
}

BLVD20KM_asukiaaa::BLVD20KM_asukiaaa(rs485_asukiaaa::ModbusRtu::Central *modbus,
                                     uint8_t address)
    : createdModbus(false) {
  this->address = address;
  this->modbus = modbus;
}

BLVD20KM_asukiaaa::~BLVD20KM_asukiaaa() {
  if (createdModbus) {
    delete modbus;
  }
}

void BLVD20KM_asukiaaa::begin(unsigned long baudrate, unsigned long config) {
  beginModbus(modbus, baudrate, config);
  beginWithoutModbus();
}

void BLVD20KM_asukiaaa::beginModbus(rs485_asukiaaa::ModbusRtu::Central *modbus,
                                    unsigned long baudrate,
                                    unsigned long config) {
  modbus->begin(baudrate, config);
  modbus->msSilentInterval = baudrate <= 9600 ? 6 : 4;
}

void BLVD20KM_asukiaaa::beginWithoutModbus() {
  writeSpeedControlMode(BLVD20KM_SPEED_MODE_USE_DIGITALS);
  writeSpeed(BLVD20KM_SPEED_MIN);
  writeStop();
}

uint8_t BLVD20KM_asukiaaa::writeSpeedControlMode(uint16_t mode) {
  uint8_t result;
  result = writeRegister(ADDR_ANALOG_MODE_L, mode);
  if (result != 0) {
    return result;
  }
  return writeConfigTrigger();  // trigger after setting ADDR_ANALOG_MODE
}

uint8_t BLVD20KM_asukiaaa::readSpeedControlMode(uint16_t *mode) {
  return readRegisters(ADDR_ANALOG_MODE_L, 1, mode);
}

uint8_t BLVD20KM_asukiaaa::writeConfigTrigger() {
  uint8_t result = writeStop();
  if (result != 0) return result;
  // return modbus.writeRegisterBy16t(address, ADDR_CONFIG_L, 1);
  return writeRegister(ADDR_CONFIG_L, 1);
}

uint16_t createMotorControl16bit(uint8_t motorDirection,
                                 bool freeLockOnStop = true,
                                 bool slowChange = true,
                                 uint8_t motorDataNum = 0) {
  // MB-FREE, -, STOP-MODE, REV, FWD, M1, M2, M0
  uint16_t bits = 0x0000;
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

uint8_t BLVD20KM_asukiaaa::writeForward() {
#ifdef DEBUG_PRINT
  Serial.println("forward");
#endif
  // uint16_t data16bit = B10101000; // forward and unlock blake
  // uint16_t data16bit = B10001000;
  // return writeRegister(ADDR_MOTOR_CONTROL, data16bit);
  return writeRegister(ADDR_MOTOR_CONTROL,
                       createMotorControl16bit(MOTOR_DIRECTOIN_FORWARD));
}

uint8_t BLVD20KM_asukiaaa::writeLock() {
#ifdef DEBUG_PRINT
  Serial.println("lock");
#endif
  return writeRegister(ADDR_MOTOR_CONTROL,
                       createMotorControl16bit(MOTOR_DIRECTOIN_STOP, false));
}

uint8_t BLVD20KM_asukiaaa::writeStop() {
#ifdef DEBUG_PRINT
  Serial.println("stop");
#endif
  return writeRegister(ADDR_MOTOR_CONTROL,
                       createMotorControl16bit(MOTOR_DIRECTOIN_STOP));
}

uint8_t BLVD20KM_asukiaaa::writeReverse() {
#ifdef DEBUG_PRINT
  Serial.println("reverse");
#endif
  return writeRegister(ADDR_MOTOR_CONTROL,
                       createMotorControl16bit(MOTOR_DIRECTOIN_REVERSE));
}

uint8_t BLVD20KM_asukiaaa::writeSpeed(uint16_t speed) {
#ifdef DEBUG_PRINT
  Serial.println("writeSpeed " + String(speed));
#endif
  return writeRegister(ADDR_SPEED0_L, speed);
}

uint8_t BLVD20KM_asukiaaa::writeTorqueLimit(uint16_t torque) {
#ifdef DEBUG_PRINT
  Serial.println("writeTorque " + String(torque));
#endif
  return writeRegister(ADDR_TORQUE_L, torque);
}

uint8_t BLVD20KM_asukiaaa::writeDiagnosis() {
  uint8_t result;
  uint8_t data[41];
  data[0] = 0;
  data[1] = 0;
  data[2] = 1;
  data[3] = 2;
  writeQuery(FN_CODE_DIAGNOSIS, data, sizeof(data));
  for (uint8_t i = 0; i < 41; ++i) {
    data[i] = 0;
  }
  result = readQuery(FN_CODE_DIAGNOSIS, data, sizeof(data));
  if (result != 0) {
    return result;
  }
  if (data[2] != 1 || data[3] != 2) {
    return BLVD20KM_ERROR_DIAGNOSIS_DATA_INVALID;
  }

  return 0;
}

uint8_t BLVD20KM_asukiaaa::writeResetAlarm() {
#ifdef DEBUG_PRINT
  Serial.println("reset alarm");
#endif
  return writeRegister(ADDR_RESET_ALARM_L, 1);
}

uint8_t BLVD20KM_asukiaaa::readAlarm(uint16_t *alarm) {
#ifdef DEBUG_PRINT
  Serial.println("read alarm");
#endif
  return readRegisters(ADDR_ALARM_L, 1, alarm);
}

uint8_t BLVD20KM_asukiaaa::readDirection(boolean *forwarding,
                                         boolean *reversing,
                                         boolean *freeLockOnStop) {
#ifdef DEBUG_PRINT
  Serial.println("read direction");
#endif
  uint16_t data;
  uint8_t result;
  result = readRegisters(ADDR_MOTOR_CONTROL, 1, &data);
  if (result != 0) {
    return result;
  }
  *forwarding = (MOTOR_FORWARD_BIT & data) != 0x00;
  *reversing = (MOTOR_REVERSE_BIT & data) != 0x00;
  *freeLockOnStop = (MOTOR_FREE_ON_STOP_BIT & data) != 0x00;
  return 0;
}

uint16_t uint8tsToUint16t(uint8_t *chars) {
  return ((uint16_t)chars[0]) << 8 | (uint16_t)chars[1];
}

uint32_t uint16tsToUint32t(uint16_t *shorts) {
  return ((uint32_t)shorts[0]) << 16 | (uint32_t)shorts[1];
}

uint8_t BLVD20KM_asukiaaa::readUint32t(uint16_t readStartAddress,
                                       uint32_t *value) {
  uint8_t result;
  result = readRegisters(readStartAddress, 2, uint16Buffer);
  if (result != 0) {
    return result;
  }
  *value = uint16tsToUint32t(uint16Buffer);
  return 0;
}

uint8_t BLVD20KM_asukiaaa::readInt32t(uint16_t readStartAddress,
                                      int32_t *value) {
  uint32_t u32;
  auto result = readUint32t(readStartAddress, &u32);
  if (result != 0) {
    return result;
  }
  memcpy(value, &u32, sizeof(u32));
  return 0;
}

uint8_t BLVD20KM_asukiaaa::readTorque(uint16_t *torque) {
  return readRegisters(ADDR_TORQUE_L, 1, torque);
}

uint8_t BLVD20KM_asukiaaa::readTorqueLimit(uint16_t *torque) {
  return readRegisters(ADDR_TORQUE_LIMIT0_L, 1, torque);
}

uint8_t BLVD20KM_asukiaaa::readSpeed(uint16_t *speed) {
  return readRegisters(ADDR_SPEED0_L, 1, speed);
}

uint8_t BLVD20KM_asukiaaa::readCommandSpeed(int32_t *speed) {
  return readInt32t(ADDR_COMMAND_SPEED_H, speed);
}

uint8_t BLVD20KM_asukiaaa::readFeedbackSpeed(int32_t *speed) {
  return readInt32t(ADDR_FEEDBACK_SPEED_H, speed);
}

uint8_t BLVD20KM_asukiaaa::readLoadTorque(uint16_t *torquePercent) {
  return readRegisters(ADDR_LOAD_FACTOR_L, 1, torquePercent);
}

uint8_t BLVD20KM_asukiaaa::writeRegister(uint16_t writeAddress,
                                         uint16_t data16bit) {
  return modbus->writeRegisterBy16t(address, writeAddress, data16bit);
}

uint8_t BLVD20KM_asukiaaa::readRegisters(uint16_t readStartAddress,
                                         uint16_t dataLen,
                                         uint16_t *registerData) {
  return modbus->readRegistersBy16t(address, readStartAddress, registerData,
                                    dataLen);
}

void BLVD20KM_asukiaaa::writeQuery(uint8_t fnCode, uint8_t *data,
                                   uint16_t dataLen) {
  modbus->writeQuery(address, fnCode, data, dataLen);
}

uint8_t BLVD20KM_asukiaaa::readQuery(uint8_t fnCode, uint8_t *data,
                                     uint16_t dataLen) {
  return modbus->readQuery(address, fnCode, data, dataLen, 20UL);
}

String BLVD20KM_asukiaaa::getStrOfError(uint8_t error) {
  return rs485_asukiaaa::ModbusRtu::getStrOfError(
      (rs485_asukiaaa::ModbusRtu::Error)error);
}

String BLVD20KM_asukiaaa::getStrOfAlarm(uint16_t alarm) {
  typedef BLVD20KM_asukiaaa_Alarm Alarm;
  switch ((Alarm)alarm) {
    case Alarm::CommunicationSwitchSettingError:
      return "Communication switch setting error";
    case Alarm::EEPROMError:
      return "EEPROM error";
    case Alarm::ExternalStop:
      return "External stop";
    case Alarm::InitialSensorError:
      return "Internal sensor error";
    case Alarm::MainCicruitOutputError:
      return "Main circuit output error";
    case Alarm::MainCicruitOverheat:
      return "Main circuit overheat";
    case Alarm::NetworkBusError:
      return "Network bus error";
    case Alarm::NetworkConverterError:
      return "NetworkConvertError";
    case Alarm::None:
      return "None";
    case Alarm::Overcurrent:
      return "Overcurrent";
    case Alarm::Overload:
      return "Overload";
    case Alarm::Overspeed:
      return "Overspeed";
    case Alarm::Overvoltage:
      return "Overvoltage";
    case Alarm::PreventionOfOperationAtPowerOn:
      return "Prevention of operation at power on";
    case Alarm::RS485CommunicationError:
      return "RS485 communicaton error";
    case Alarm::RS485CommunicationTimeout:
      return "RS485 communication timeout";
    case Alarm::SensorError:
      return "Sensor error";
    case Alarm::Undervoltage:
      return "Undervoltage";
  }
  return "";
}
