#include <Arduino.h>
#include <config.h>
#include <digital_pid.h>
#include <Servo.h>
#include <Adafruit_SSD1306.h>
#include <driver_motors.h>

namespace DigitalPID {
  
  // Define Servo
  Servo servo;

  // Initializing PID controller
  PID pid = {
    .Kp                 = 40.0f,
    .Ki                 = 0.0f,
    .Kd                 = 10.0f,
    .L_THRESHOLD        = 600.0f,   // ADC Threshold voltage (0-1023 analog maps to 0 - 3.3V)
    .R_THRESHOLD        = 600.0f,   // ADC Threshold voltage (0-1023 analog maps to 0 - 3.3V)
                                    // On white approx 600
                                    // On black approx 350-450
    .STRAIGHT_ANGLE     = 90,       // 90 degrees for servo is straight forward
    .MAX_INTEGRAL       = 100.0f,
    .leftInput          = 0.0f,     // Current left-wheel reading
    .rightInput         = 0.0f,     // Current right-wheel reading
    .error              = 0.0f,     // PID proportional term
    .prevError          = 0.0f,     // Previous error for derivative term
    .derivative         = 0.0f,     // PID derivative term
    .integral           = 0.0f,     // PID integral term
    .output             = 0.0f,     // PID output
    .currTime           = 0,
    .prevTime           = 0,
    .dt                 = 0,
    .MAX_ANGLE          = 50,
    .MIN_ANGLE          = -50,
  };

  /*
    @brief attaches servo to specified pin
  */
  void setupServo() {
    servo.attach(STEERING_SERVO);
  }

  /*
    @brief Applies PID calculations and 
    converts sensor inputs to servo angles
  */

  // TODO: change return type back to void?
  String applyPID() {
    // Calculate the elapsed time since the last iteration
    pid.currTime = millis();
    pid.dt = pid.currTime - pid.prevTime;

    // Read the current value (Low on Tape, High elsewhere)
    pid.leftInput = analogRead(LEFT_TAPE_PIN);
    pid.rightInput = analogRead(RIGHT_TAPE_PIN);

    // Calculate the error term
    // TODO: interrupt with IR when not seeing tape
    calcError(&pid.leftInput, &pid.rightInput);

    // Calculate the integral term
    pid.integral += pid.error * pid.dt;
    // We want to cap out integral contribution in case we have constant error
    if(pid.integral > pid.MAX_INTEGRAL) {
      pid.integral = pid.MAX_INTEGRAL;
    }
    else if (pid.integral < pid.MIN_ANGLE) {
      pid.integral = pid.MIN_ANGLE;
    }

    // Calculate the derivative term
    pid.derivative = (pid.error - pid.prevError) / pid.dt;

    // Calculate the PID output
    pid.output = pid.Kp * pid.error + pid.Ki * pid.integral + pid.Kd * pid.derivative;

    // Update the previous error and time for the next iteration
    pid.prevError = pid.error;
    pid.prevTime = pid.currTime;

    // Apply the output (e.g. turn servo)
    // String *out;
    return processOutput(&pid.output);
    // result = out;
  }

  /*
    @brief Calculates the error via ADC. Helper function
  */
  static void calcError(float_t *left, float_t *right) {
    
    /*
      -1: Left is off tape
      0: both on tape
      1: right is on tape
      2: both off tape
    */

    if(*left > pid.L_THRESHOLD && *right < pid.R_THRESHOLD) {
      // Left is off tape and Right is on tape
      // TURN RIGHT
      pid.error = -1.0f;
    } else if(*left < pid.L_THRESHOLD && *right > pid.R_THRESHOLD) {
      // Left is on tape and Right is off tape
      // TURN LEFT
      pid.error = 1.0f;
    } else if(*left > pid.L_THRESHOLD && *right > pid.R_THRESHOLD) {
      // Left is off tape and Right is off tape
      // Look at previous error state and magnify it
      pid.error = pid.prevError * 2.0f;
    } else {
      pid.error = 0.0f;
    }
  }

  /*
    @brief Processes the PID calculated output and determines
    how the servo needs to move in response. 90 is forwards,
    anything less is a right turn
  */
  static String processOutput(float_t *output) {

    //Limits output angle
    if(*output > pid.MAX_ANGLE) {
      *output = pid.MAX_ANGLE;
    } else if(*output < pid.MIN_ANGLE) {
      *output = pid.MIN_ANGLE;
    }

    // Process servo angle from PID output
    if(*output < 0) {
      int8_t duty_cycle = 0;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      servo.write(pid.STRAIGHT_ANGLE - *output);
      return "Turn right (O: " + String(*output) + ")";
    } else if(*output > 0) {
      int8_t duty_cycle = 0;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      servo.write(pid.STRAIGHT_ANGLE - *output);
      return "Turn left (O: " + String(*output) + ")";
    } else {
      int8_t duty_cycle = 0;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      servo.write(pid.STRAIGHT_ANGLE + *output);
      return "Go straight (O: " + String(*output) + ")";
    }
  }
}
