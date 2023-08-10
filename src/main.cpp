#include <config.h>

// Setting up TX RX pins for Serial2
HardwareSerial Serial2(PA3, PA2);

void setup() {
  Hivemind::setupHivemind();
}

void loop() {
  // Serial2.println(SensorFusion::IMUGetData()[1]);

  // Halt if START_LAP pin is low
  if(digitalRead(START_LAP) == HIGH){
    Hivemind::wakeUpHivemind();
  } else {
    SensorFusion::IMUGetData();
  }

  // digitalWrite(PC13, LOW);
  // delay(50); 
  // digitalWrite(PC13, HIGH);
  delay(20);

}
