#include <BLVD20KM_asukiaaa.h>

#define RS485_DE 4
#define RS485_RE 5
#define MOTOR_ADDRESS 0 // 0 for udp

BLVD20KM_asukiaaa motor(&Serial1, MOTOR_ADDRESS, RS485_DE, RS485_RE);

uint16_t speed, alarmState;

void setup() {
  Serial.begin(115200);
  motor.begin(115200);
}

void readAndPrintSpeed() {
  if (motor.readSpeed(&speed) == 0) {
    Serial.println("Speed is " + String(speed));
  } else {
    Serial.println("Cannot read speed");
  }
}

void loop() {
  motor.writeSpeed(500);
  motor.writeForward();
  readAndPrintSpeed();
  delay(2000);

  motor.writeStop();
  delay(1000);

  motor.writeSpeed(200);
  motor.writeReverse();
  readAndPrintSpeed();
  delay(1000);

  motor.writeSpeed(500);
  readAndPrintSpeed();
  delay(1000);

  motor.writeStop();
  if (motor.readAlarm(&alarmState) == 0) {
    Serial.println("Current alarm: " + String(alarmState));
  } else {
    Serial.println("Cannot read alarm");
  }
  delay(1000);
}
