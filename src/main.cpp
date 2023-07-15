#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// Custom header files
#include <fft.h>
#include <config.h>
#include <digital_pid.h>
#include <driver_motors.h>


// Set up screen dimensions
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Testing variables
uint32_t counter = 0;
// float_t L_THRESHOLD = 600;
// float_t R_THRESHOLD = 600;
// Servo servo;
// int16_t previousError = 0;

void setup() {   
  //setup servo
  DigitalPID::setupServo();
  // servo.attach(STEERING_SERVO);

  //setup Oled
  //OledDisplay::setupDisplay();
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.display();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);

  delay(1000);
}

void loop() {
  display_handler.clearDisplay();
  display_handler.setCursor(0, 0);
  // String *result;
  // DigitalPID::applyPID(result);
  display_handler.println(DigitalPID::applyPID());
  display_handler.println(counter++);

  // for(int i = 45; i < 130; i++) {
  //     display_handler.clearDisplay();
  //     display_handler.setTextSize(1);
  //     display_handler.setTextColor(SSD1306_WHITE);
  //     display_handler.setCursor(0, 0);
  //     display_handler.println("Servo angle ");
  //     display_handler.println(i);
  //     servo.write(i);
  //     display_handler.display();
  //     delay(100);
  // }

  //Max is 35

  // int i = 90;
  // display_handler.clearDisplay();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);
  // display_handler.setCursor(0, 0);
  // display_handler.println("Servo angle ");
  // display_handler.println(i);
  // servo.write(i);
  // display_handler.display();
  // delay(1000);

  // float_t left = analogRead(LEFT_TAPE_PIN);
  // float_t right = analogRead(RIGHT_TAPE_PIN);

  // display_handler.clearDisplay();
  // display_handler.setTextSize(1);
  // display_handler.setTextColor(SSD1306_WHITE);
  // display_handler.setCursor(0, 0);
  // display_handler.print("Left: ");
  // display_handler.println(left);
  // display_handler.print("Right: ");
  // display_handler.println(right);

  // int16_t error = func(left,right);

  // float_t derivative = (error - previousError) / 100;

  // float_t output = 50.0f * error + 5.0f * derivative;
  // display_handler.print("Error:");
  // display_handler.println(error);
  // display_handler.print("Output:");
  // display_handler.println(output);
  // display_handler.println(processOutput(output));
  // display_handler.println(counter++);
  

  // if(left > L_THRESHOLD && right < R_THRESHOLD){
  //   //Left is off tape and Right is on tape
  //     display_handler.println("TURN RIGHT");

  // }
  // else if(left < L_THRESHOLD && right > R_THRESHOLD){
  //   //left is on tape and Right is off tape
  //     display_handler.println("TURN LEFT");

  // }
  // // else if(left > L_THRESHOLD && right > R_THRESHOLD){
  // //   //Left is off tape and Right is off tape
  // //   // TODO Look at previous error state and magnify it
  // //   return previousError * 2;
  // // }
  // else{
  //     display_handler.println("GO STRAIGHT");
  // }

  if(counter > 1000) {
      counter = 0;
  }

  // previousError = error;
  display_handler.display();
  
  delay(50);
}