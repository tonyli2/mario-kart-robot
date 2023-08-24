#include <config.h>

// Setting up TX RX pins for Serial2
HardwareSerial Serial2(PA3, PA2);

void setup() {
  Hivemind::setupHivemind();
}

void loop() {

  // Allows us to trigger a "hot start"
  if(digitalRead(START_LAP) == HIGH){
    Hivemind::wakeUpHivemind();
  } else {
    // Keep IMU data updated
    SensorFusion::IMUGetData();
  }

  delay(20);

}
