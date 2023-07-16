#include <Arduino.h>
#include <config.h>
#include <digital_pid.h>
#include <Adafruit_SSD1306.h>
#include <driver_motors.h>

namespace DigitalPID {

  // Initializing steering PID controller
  PID steering_pid = {
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
    steering_pid.servo.attach(STEERING_SERVO);
    // TODO: set up servo for IR sensor
  }

  /*
    @brief Applies PID calculations and 
    converts sensor inputs to servo angles
  */

  // TODO: change return type back to void?
  String applySteeringPID() {
    // Calculate the elapsed time since the last iteration
    steering_pid.currTime = millis();
    steering_pid.dt = steering_pid.currTime - steering_pid.prevTime;

    // Read the current value (Low on Tape, High elsewhere)
    steering_pid.leftInput = analogRead(LEFT_TAPE_PIN);
    steering_pid.rightInput = analogRead(RIGHT_TAPE_PIN);

    // Calculate the error term
    // TODO: interrupt with IR when not seeing tape
    calcSteeringError(&steering_pid.leftInput, &steering_pid.rightInput);

    // Calculate the integral term
    steering_pid.integral += steering_pid.error * steering_pid.dt;
    // We want to cap out integral contribution in case we have constant error
    if(steering_pid.integral > steering_pid.MAX_INTEGRAL) {
      steering_pid.integral = steering_pid.MAX_INTEGRAL;
    } else if (steering_pid.integral < steering_pid.MIN_ANGLE) {
      steering_pid.integral = steering_pid.MIN_ANGLE;
    }

    // Calculate the derivative term
    steering_pid.derivative = (steering_pid.error - steering_pid.prevError) / steering_pid.dt;

    // Calculate the PID output
    steering_pid.output = steering_pid.Kp * steering_pid.error + steering_pid.Ki * steering_pid.integral + steering_pid.Kd * steering_pid.derivative;

    // Update the previous error and time for the next iteration
    steering_pid.prevError = steering_pid.error;
    steering_pid.prevTime = steering_pid.currTime;

    // Apply the output (e.g. turn servo)
    // String *out;
    return processSteeringOutput(&steering_pid.output);
    // result = out;
  }

  /*
    @brief Calculates the error via ADC. Helper function
  */
  static void calcSteeringError(float_t *left, float_t *right) {
    
    /*
      -1: Left is off tape
      0: both on tape
      1: right is on tape
      2: both off tape
    */

    if(*left > steering_pid.L_THRESHOLD && *right < steering_pid.R_THRESHOLD) {
      // Left is off tape and Right is on tape
      // TURN RIGHT
      steering_pid.error = -1.0f;
    } else if(*left < steering_pid.L_THRESHOLD && *right > steering_pid.R_THRESHOLD) {
      // Left is on tape and Right is off tape
      // TURN LEFT
      steering_pid.error = 1.0f;
    } else if(*left > steering_pid.L_THRESHOLD && *right > steering_pid.R_THRESHOLD) {
      // Left is off tape and Right is off tape
      // Look at previous error state and magnify it
      steering_pid.error = steering_pid.prevError * 2.0f;
    } else {
      steering_pid.error = 0.0f;
    }
  }

  /*
    @brief Processes the PID calculated output and determines
    how the servo needs to move in response. 90 is forwards,
    anything less is a right turn
  */
  static String processSteeringOutput(float_t *output) {

    //Limits output angle
    if(*output > steering_pid.MAX_ANGLE) {
      *output = steering_pid.MAX_ANGLE;
    } else if(*output < steering_pid.MIN_ANGLE) {
      *output = steering_pid.MIN_ANGLE;
    }

    // Process servo angle from PID output
    if(*output < 0) {
      int8_t duty_cycle = 15;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      steering_pid.servo.write(steering_pid.STRAIGHT_ANGLE - *output);
      return "Turn right (O: " + String(*output) + ")";
    } else if(*output > 0) {
      int8_t duty_cycle = 15;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      steering_pid.servo.write(steering_pid.STRAIGHT_ANGLE - *output);
      return "Turn left (O: " + String(*output) + ")";
    } else {
      int8_t duty_cycle = 40;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      steering_pid.servo.write(steering_pid.STRAIGHT_ANGLE + *output);
      return "Go straight (O: " + String(*output) + ")";
    }
  }
}
