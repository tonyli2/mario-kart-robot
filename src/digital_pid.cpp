#include <Arduino.h>
#include <config.h>
#include <digital_pid.h>
#include <Servo.h>
#include <Adafruit_SSD1306.h>

namespace DigitalPID {

  
  //Define Servo
  Servo servo;

  // Define constants
  const float Kp = 5.0;  // Proportional gain
  const float Ki = 0;  // Integral gain
  const float Kd = 0;  // Derivative gain
  float L_THRESHOLD = 600;  // ADC Threshold voltage (0-1023 analog maps to 0 - 3.3V)
  float R_THRESHOLD = 600;
  const u_int8_t STRAIGHT_ANGLE = 90; //90 degrees for servo is straight forward
  const float MAX_INTEGRAL = 100;
  //On white approx 600
  //On black approx 350-450

  // Define PID variables
  double leftInput = 0;  // Current left-wheel reading
  double rightInput = 0; // Current right-wheel reading
  double output = 0;     // Output value

  // Define variables for calculation
  short error = 0;          // Current error term
  short previousError = 0;  // Previous error term
  double integral = 0;      // Integral term

  // Define time variables
  uint64_t previousTime = 0;  // Previous time for derivative calculation
  uint64_t sampleTime = 100;  // Sample time in milliseconds

  //Angle variables
  const u_int8_t MAX_ANGLE = 50;
  const u_int8_t MIN_ANGLE = -50;

  /*
    @brief attaches servo to specified pin
  */
  void setupServo(){
    servo.attach(STEERING_SERVO);
  }

  /*
    @brief Applies PID calculations and 
    converts sensor inputs to servo angles
  */

  //TODO change return type back to void?
  String applyPID() {
    // Calculate the elapsed time since the last iteration
    uint64_t currentTime = millis();
    uint64_t elapsedTime = currentTime - previousTime;

    // Check if the sample time has passed
    if (elapsedTime >= sampleTime) {

      // Read the current value (Low on Tape, High elsewhere)
      leftInput = analogRead(LEFT_TAPE_PIN);
      rightInput = analogRead(RIGHT_TAPE_PIN);

      // Calculate the error term
      // TODO: interrupt with IR when not seeing tape
      error = calcError(leftInput, rightInput);

      // Calculate the integral term
      integral += error * elapsedTime;
      // We want to cap out integral contribution in case we have constant error
      if(integral > MAX_INTEGRAL){
        integral = MAX_INTEGRAL;
      }
      else if (integral < -1 * MAX_INTEGRAL){
        integral = -1 * MAX_INTEGRAL;
      }

      // Calculate the derivative term
      double derivative = (error - previousError) / elapsedTime;

      // Calculate the PID output
      output = Kp * error + Ki * integral + Kd * derivative;

      // Update the previous error and time for the next iteration
      previousError = error;
      previousTime = currentTime;

      // Apply the output (e.g. turn servo)
      return processOutput(output);
    }
  }

  /*
    @brief Calculates the error via ADC. Helper function
  */
  static short calcError(double left, double right){
    
    /*
      -1: Left is off tape
      0: both on tape
      1: right is on tape
      2: both off tape
    */

    if(left > L_THRESHOLD && right < R_THRESHOLD){
      //Left is off tape and Right is on tape
      //TURN RIGHT
      return -1;
    }
    else if(left < L_THRESHOLD && right > R_THRESHOLD){
      //left is on tape and Right is off tape
      //TURN LEFT
      return 1;
    }
    // else if(left > L_THRESHOLD && right > R_THRESHOLD){
    //   //Left is off tape and Right is off tape
    //   // TODO Look at previous error state and magnify it
    //   return previousError * 2;
    // }
    else{
      return 0;
    }
  }

  /*
    @brief Processes the PID calculated output and determines
    how the servo needs to move in response. 90 is forwards,
    anything less is a right turn
  */
  static String processOutput(double output) {

    //Limits output angle
    if(output > MAX_ANGLE){
      output = MAX_ANGLE;
    }
    else if(output < MIN_ANGLE){
      output = MIN_ANGLE;
    }

    //Process servo angle from PID output
    if(output < 0){
      servo.write(STRAIGHT_ANGLE + output);
      return "Turn right (O: " + String(output) + ")";
    }
    else if(output > 0){
      servo.write(STRAIGHT_ANGLE + output);
      return "Turn left (O: " + String(output) + ")";
    }
    else {
      servo.write(STRAIGHT_ANGLE + output);
      return "Go straight (O: " + String(output) + ")";
    }
  }
}
