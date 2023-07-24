#include <config.h>

// Set up screen dimensions
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Testing variables
uint32_t counter = 0;

void printIMU(float_t *attitude_vec);

void setup() {

  //Setup Serial Monitor
  Serial.begin(9600);


  // Set up OLED
  // display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // display_handler.display();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);

  int8_t IMUReady = SensorFusion::IMUInit();
  if (IMUReady > 0) {
    display_handler.println("IMU READY!!!");
    delay(1000);
  }

  display_handler.display();
  delay(1000);
}

void loop() {

  Serial.println(counter++);

  if(counter > 1000) {
      counter = 0;
  }

  
  delay(1000);
}

void printIMU(float_t *attitude_vec) {
  display_handler.print("Roll:");
  display_handler.println(attitude_vec[0]);
  display_handler.print("Pitch:");
  display_handler.println(attitude_vec[1]);
  display_handler.print("Yaw:");
  display_handler.println(attitude_vec[2]);
}
