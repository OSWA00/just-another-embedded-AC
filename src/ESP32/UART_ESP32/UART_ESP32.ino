#include <RBDdimmer.h>

#define outPin 26
#define Zero   27

dimmerLamp dimmer(outPin, Zero);

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200, SERIAL_8E1);
  dimmer.begin(NORMAL_MODE, ON);
}

void loop() {
  if(Serial1.available()){
    dimmer.setPower(Serial1.read());
  }
}
