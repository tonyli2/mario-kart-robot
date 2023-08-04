#include <config.h>

// Setting up TX RX pins for Serial2
HardwareSerial Serial2(PA3, PA2);

void setup() {
  Hivemind::setupHivemind();
}

void loop() {
  
  Hivemind::wakeUpHivemind();

  // Hivemind::testMotors();

  // digitalWrite(PC13, LOW);
  // delay(50); 
  // digitalWrite(PC13, HIGH);
  delay(100);

}
