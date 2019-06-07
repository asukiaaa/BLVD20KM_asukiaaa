#ifndef BLVD20KM_H
#define BLVD20KM_H
#include <Arduino.h>
#include <HardwareSerial.h>

#define BLVD20KM_QUERY_MAX_LEN 41

#define BLVD20KM_ERROR_CODE_INVALID_FN        0x01
#define BLVD20KM_ERROR_CODE_INVALID_ADDR      0x02
#define BLVD20KM_ERROR_CODE_INVALID_DATA      0x03
#define BLVD20KM_ERROR_CODE_SLAVE_ERROR       0x04
#define BLVD20KM_ERROR_NO_RESPONSE            0x10
#define BLVD20KM_ERROR_UNMATCH_CRC            0x11
#define BLVD20KM_ERROR_UNMATCH_ADDRESS        0x12
#define BLVD20KM_ERROR_UNMATCH_FN_CODE        0x13
#define BLVD20KM_ERROR_UNMATCH_DATA_LEN       0x14
#define BLVD20KM_ERROR_OVER_QUERY_MAX_LEN     0x15
#define BLVD20KM_ERROR_DIAGNOSIS_DATA_INVALID 0x16

#define BLVD20KM_SPEED_MIN 80
#define BLVD20KM_SPEED_MAX 4000

#define BLVD02KM_SPEED_MODE_USE_DIGITALS 0x0001

class BLVD20KM_asukiaaa {
 public:
  BLVD20KM_asukiaaa(HardwareSerial* serial, unsigned char address, unsigned char dePin, unsigned char rePin);
// #ifndef __arm__
//   BLVD20KM_asukiaaa(SoftwareSerial* serial, unsigned char address, unsigned char dePin, unsigned char rePin);
// #endif

  void begin();

  unsigned char writeForward();
  unsigned char writeLock();
  unsigned char writeStop();
  unsigned char writeReverse();

  unsigned char writeDiagnosis();
  unsigned char readSpeed(unsigned short *speed);
  unsigned char readSpeedControlMode(unsigned short *mode);
  unsigned char readDirection(boolean *forwarding, boolean *reversing, boolean *freeLockOnStop);
  unsigned char writeSpeed(unsigned short speed);
  unsigned char writeSpeedControlMode(unsigned short mode);

 private:
  HardwareSerial* serial;
  unsigned char address;
  unsigned char dePin;
  unsigned char rePin;
  unsigned char queryBuffer[BLVD20KM_QUERY_MAX_LEN];
  unsigned char readQuery(unsigned char fnCode, unsigned char* data, unsigned short dataLen);
  unsigned char readRegisters(unsigned short readStartAddress, unsigned short dataLen, unsigned short* registerData);
  unsigned char writeConfigTrigger();
  unsigned char writeRegister(unsigned short writeAddress, unsigned short data16bit);
  void writeQuery(unsigned char fnCode, unsigned char* data, unsigned int dataLen);
};

#endif
