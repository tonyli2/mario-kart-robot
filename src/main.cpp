#include <config.h>

// Set up screen dimensions
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup() {
  // pinMode(PC13, OUTPUT);
  Hivemind::setupHivemind();

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.display();
  delay(100);
}

void loop() {

  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  display_handler.print("Left Freq: ");
  display_handler.println(String(FFT::runFFT(IR_DETECTOR_LEFT)[0]) + " At " + String(FFT::runFFT(IR_DETECTOR_LEFT)[1]));
  display_handler.print("Right Freq: ");
  display_handler.println(String(FFT::runFFT(IR_DETECTOR_RIGHT)[0]) + " At " + String(FFT::runFFT(IR_DETECTOR_RIGHT)[1]));

  display_handler.println(String(FFT::hasFoundBeacon(IR_DETECTOR_LEFT, IR_DETECTOR_RIGHT)));
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
  Hivemind::testIR();
  // Hivemind::wakeUpHivemind();
  
  
  // digitalWrite(PC13, LOW);
  // delay(50);
  // digitalWrite(PC13, HIGH);
  delay(100);

  // Hivemind::testServo(90);
}
