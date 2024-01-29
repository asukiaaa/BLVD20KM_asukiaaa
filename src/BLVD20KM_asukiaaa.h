#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <rs485_asukiaaa.h>

#include "./BLVD20KM_asukiaaa/version.h"

#define BLVD20KM_ERROR_DIAGNOSIS_DATA_INVALID 0x16

#define BLVD20KM_SPEED_MIN 80
#define BLVD20KM_SPEED_MAX 4000
#define BLVD20KM_TORQUE_MAX 200

#define BLVD20KM_SPEED_MODE_USE_DIGITALS 0x0001

enum class BLVD20KM_asukiaaa_Alarm : uint8_t {
  None = 0,
  Overload = 0x30,
  SensorError = 0x28,
  InitialSensorError = 0x42,
  Overvoltage = 0x22,
  Undervoltage = 0x25,
  Overspeed = 0x31,
  Overcurrent = 0x20,
  EEPROMError = 0x41,
  MainCicruitOverheat = 0x21,
  ExternalStop = 0x6e,
  PreventionOfOperationAtPowerOn = 0x46,
  NetworkBusError = 0x81,
  CommunicationSwitchSettingError = 0x83,
  RS485CommunicationError = 0x84,
  RS485CommunicationTimeout = 0x85,
  NetworkConverterError = 0x8e,
  MainCicruitOutputError = 0x2d,
};

class BLVD20KM_asukiaaa {
 public:
  rs485_asukiaaa::ModbusRtu::Central *modbus;
  BLVD20KM_asukiaaa(HardwareSerial *serial, uint8_t address, uint8_t dePin,
                    uint8_t rePin);
  BLVD20KM_asukiaaa(rs485_asukiaaa::ModbusRtu::Central *modbus,
                    uint8_t address);
  ~BLVD20KM_asukiaaa();
  void begin(unsigned long baudrate, unsigned long config = SERIAL_8E1);
  void beginWithoutModbus();

  uint8_t writeForward();
  uint8_t writeLock();
  uint8_t writeStop();
  uint8_t writeReverse();

  uint8_t readCommandSpeed(int32_t *speed);
  uint8_t readDirection(bool *forwarding, bool *reversing,
                        bool *freeLockOnStop);
  uint8_t readFeedbackSpeed(int32_t *speed);
  uint8_t readLoadTorque(uint16_t *speed);
  uint8_t readSpeed(uint16_t *speed);
  uint8_t readSpeedControlMode(uint16_t *mode);
  uint8_t readTorque(uint16_t *torque);
  uint8_t readTorqueLimit(uint16_t *torque);
  uint8_t writeSpeed(uint16_t speed);
  uint8_t writeSpeedControlMode(uint16_t mode);
  uint8_t writeSpeedControlModeIfDifferent(uint16_t mode);
  uint8_t writeTorqueLimit(uint16_t torque);

  uint8_t writeDiagnosis();
  uint8_t readAlarm(uint16_t *alarm);
  uint8_t writeResetAlarm();

  static String getStrOfAlarm(uint16_t alarm);
  static String getStrOfError(uint8_t error);
  static void beginModbus(rs485_asukiaaa::ModbusRtu::Central *modbus,
                          unsigned long baudrate,
                          unsigned long config = SERIAL_8E1);
  static unsigned long getMsSilentInterval(unsigned long baudrate);

 private:
  const bool createdModbus;
  uint8_t address;
  uint8_t readInt32t(uint16_t readStartAddress, int32_t *value);
  uint8_t readUint32t(uint16_t readStartAddress, uint32_t *value);
  uint8_t readQuery(uint8_t fnCode, uint8_t *data, uint16_t dataLen);
  uint8_t readRegisters(uint16_t readStartAddress, uint16_t dataLen,
                        uint16_t *registerData);
  uint8_t writeConfigTrigger();
  uint8_t writeRegister(uint16_t writeAddress, uint16_t data16bit);
  void writeQuery(uint8_t fnCode, uint8_t *data, uint16_t dataLen);
  static uint16_t uint8tsToUint16t(uint8_t *chars);
  static uint32_t uint16tsToUint32t(uint16_t *shorts);

  uint16_t uint16Buffer[8];
  uint8_t uint8Buffer[41];
};
