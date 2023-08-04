#include <config.h>

// Set up screen dimensions
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void printIMU(float_t *attitude_vec);
void SpeedToAngle(uint32_t *speed, float_t *angle);

bool driveStraight = false;
float32_t yawHistory[10] = {0.0f};
uint8_t yawCounter = 0;
uint32_t speed = 80;

void setup() {
  // pinMode(PC13, OUTPUT);
  Hivemind::setupHivemind();

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);

  int8_t IMUReady = SensorFusion::IMUInit();
  if (IMUReady > 0) {
    display_handler.println("IMU READY!!!");
    // delay(1000);
  }

  display_handler.print("Speed is ");
  display_handler.println(speed);

  pinMode(PA8, OUTPUT);

  display_handler.display();
  delay(1000);
}

void loop() {

  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  float_t *attitude_vec = SensorFusion::IMUGetData();
  printIMU(attitude_vec);

  // yawHistory[yawCounter] = attitude_vec[1];
  // yawCounter++;
  // if (yawCounter == 10) yawCounter = 0;

  // uint8_t yawOk = 0;

  // for (float_t yawReading : yawHistory) {
  //   if (abs(yawReading) > 7.0f) {
  //     yawOk++;
  //   }
  // }

  // if (yawOk >= 5) {
  //   digitalWrite(PA8, HIGH);
  // } else {
  //   digitalWrite(PA8, LOW);
  // }

  // if (abs(attitude_vec[1]) > 7) {
  //   digitalWrite(PA8, HIGH);
  // } else {
  //   digitalWrite(PA8, LOW);
  // }

  if (driveStraight) {
    Hivemind::testServo(90);
    DriverMotors::startMotorsForwardLeft(speed);
    DriverMotors::startMotorsForwardRight(speed);
    delay(500);
    driveStraight = false;
  }

  float_t angle;
  SpeedToAngle(&speed, &angle);
  if (attitude_vec[2] <= 165) {
    Hivemind::testServo(117);
    DriverMotors::startMotorsForwardLeft(35);
    DriverMotors::startMotorsForwardRight(55);
  }
  else {
    Hivemind::testServo(90);
    DriverMotors::stopMotorsLeft();
    DriverMotors::stopMotorsRight();
  }

  // display_handler.print("Left Freq: ");
  // display_handler.println(String(FFT::runFFT(IR_DETECTOR_LEFT)[0]) + " At " + String(FFT::runFFT(IR_DETECTOR_LEFT)[1]));
  // display_handler.print("Right Freq: ");
  // display_handler.println(String(FFT::runFFT(IR_DETECTOR_RIGHT)[0]) + " At " + String(FFT::runFFT(IR_DETECTOR_RIGHT)[1]));

  // display_handler.println(String(FFT::hasFoundBeacon(IR_DETECTOR_LEFT, IR_DETECTOR_RIGHT)));
  // display_handler.print("Left Marker: ");
  // display_handler.println(analogRead(MARKER_SENSE_LEFT));
  // display_handler.print("Left Tape: ");
  // display_handler.println(analogRead(LEFT_TAPE_PIN));
  // display_handler.print("Right Tape: ");
  // display_handler.println(analogRead(RIGHT_TAPE_PIN));
  // display_handler.print("Right Marker: ");
  // display_handler.println(analogRead(MARKER_SENSE_RIGHT));
  display_handler.display();
  // Hivemind::testServo(90);
  // Hivemind::testIR();
  // Hivemind::irHivemind();
  
  
  // digitalWrite(PC13, LOW);
  // delay(50);
  // digitalWrite(PC13, HIGH);
  delay(50);

  // Hivemind::testServo(90);
}

void printIMU(float_t *attitude_vec) {
  display_handler.print("Roll:");
  display_handler.println(attitude_vec[0]);
  display_handler.print("Pitch:");
  display_handler.println(attitude_vec[1]);
  display_handler.print("Yaw:");
  display_handler.println(attitude_vec[2]);
}

void SpeedToAngle(uint32_t *speed, float_t *angle) {
  *angle = 180.0f - (0.5006f * *speed - 14.176f) - 15.0f;
}
