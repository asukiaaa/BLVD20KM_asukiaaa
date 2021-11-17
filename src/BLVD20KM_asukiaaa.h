#ifndef BLVD20KM_H
#define BLVD20KM_H
#include <Arduino.h>
#include <HardwareSerial.h>

// #define BLVD20KM_ASUKIAAA_USE_RS485_ASUKIAAA

#ifdef BLVD20KM_ASUKIAAA_USE_RS485_ASUKIAAA
#include <rs485_asukiaaa.h>
#else
#define BLVD20KM_QUERY_MAX_LEN 41
#endif

#define BLVD20KM_ERROR_CODE_INVALID_FN 0x01
#define BLVD20KM_ERROR_CODE_INVALID_ADDR 0x02
#define BLVD20KM_ERROR_CODE_INVALID_DATA 0x03
#define BLVD20KM_ERROR_CODE_SLAVE_ERROR 0x04
#define BLVD20KM_ERROR_NO_RESPONSE 0x10
#define BLVD20KM_ERROR_UNMATCH_CRC 0x11
#define BLVD20KM_ERROR_UNMATCH_ADDRESS 0x12
#define BLVD20KM_ERROR_UNMATCH_FN_CODE 0x13
#define BLVD20KM_ERROR_UNMATCH_DATA_LEN 0x14
#define BLVD20KM_ERROR_OVER_QUERY_MAX_LEN 0x15
#define BLVD20KM_ERROR_DIAGNOSIS_DATA_INVALID 0x16

#define BLVD20KM_SPEED_MIN 80
#define BLVD20KM_SPEED_MAX 4000
#define BLVD20KM_TORQUE_MAX 200

#define BLVD02KM_SPEED_MODE_USE_DIGITALS 0x0001

class BLVD20KM_asukiaaa {
 public:
  BLVD20KM_asukiaaa(HardwareSerial *serial, uint8_t address, uint8_t dePin,
                    uint8_t rePin);
  void begin(int baudrate, int config = SERIAL_8E1);

  uint8_t writeForward();
  uint8_t writeLock();
  uint8_t writeStop();
  uint8_t writeReverse();

  uint8_t readCommandSpeed(int32_t *speed);
  uint8_t readDirection(boolean *forwarding, boolean *reversing,
                        boolean *freeLockOnStop);
  uint8_t readFeedbackSpeed(int32_t *speed);
  uint8_t readLoadTorque(uint16_t *speed);
  uint8_t readSpeed(uint16_t *speed);
  uint8_t readSpeedControlMode(uint16_t *mode);
  uint8_t readTorque(uint16_t *torque);
  uint8_t readTorqueLimit(uint16_t *torque);
  uint8_t writeSpeed(uint16_t speed);
  uint8_t writeSpeedControlMode(uint16_t mode);
  uint8_t writeTorqueLimit(uint16_t torque);

  uint8_t writeDiagnosis();
  uint8_t readAlarm(uint16_t *alarm);
  uint8_t writeResetAlarm();

 private:
#ifdef BLVD20KM_ASUKIAAA_USE_RS485_ASUKIAAA
  rs485_asukiaaa::ModbusRtu::Central* modbus;
#else
  HardwareSerial* serial;
  uint8_t dePin;
  uint8_t rePin;
  uint8_t queryBuffer[BLVD20KM_QUERY_MAX_LEN];
#endif
  uint8_t address;
  uint8_t readInt32t(uint16_t readStartAddress, int32_t *value);
  uint8_t readUint32t(uint16_t readStartAddress, uint32_t *value);
  uint8_t readQuery(uint8_t fnCode, uint8_t *data, uint16_t dataLen);
  uint8_t readRegisters(uint16_t readStartAddress, uint16_t dataLen,
                        uint16_t *registerData);
  uint8_t writeConfigTrigger();
  uint8_t writeRegister(uint16_t writeAddress, uint16_t data16bit);
  void writeQuery(uint8_t fnCode, uint8_t *data, uint16_t dataLen);

  uint16_t uint16Buffer[8];
  uint8_t uint8Buffer[41];
};

#endif
