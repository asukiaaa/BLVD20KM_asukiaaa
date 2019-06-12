#include <BLVD20KM_asukiaaa.h>

#define RS485_DE 4
#define RS485_RE 5

BLVD20KM_asukiaaa motor(&Serial1, 0, RS485_DE, RS485_RE);

uint16_t speed;

void setup() {
  Serial.begin(115200);
  motor.begin();
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
  delay(1000);
}
