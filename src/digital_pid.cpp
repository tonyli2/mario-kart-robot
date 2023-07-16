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
    .L_THRESHOLD        = 600.0f,
    .R_THRESHOLD        = 600.0f,
    .STRAIGHT_ANGLE     = 90,
    .MAX_INTEGRAL       = 100.0f,
    .leftInput          = 0.0f,
    .rightInput         = 0.0f,
    .error              = 0.0f,
    .prevError          = 0.0f,
    .derivative         = 0.0f,
    .integral           = 0.0f,
    .output             = 0.0f,
    .currTime           = 0,
    .prevTime           = 0,
    .dt                 = 0,
    .MAX_ANGLE          = 50,
    .MIN_ANGLE          = -50,
  };

  /**
   * @brief Attaching servo to the specified pin
   * 
   */
  void setupServo() {
    steering_pid.servo.attach(STEERING_SERVO);
    // TODO: set up servo for IR sensor
  }

  /**
   * @brief Applying PID calculations and converting sensor inputs to servo angles
   * @return Turning direction and PID output in string form
   * 
   * @note Might change return type to void once finishing PID tunning
   */
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
    // Limit integral contribution in case we have constant error
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
    return processSteeringOutput(&steering_pid.output);
  }

  /**
   * @brief Helper function to calculate the error via ADC
   * @param[in] left Left tape sensor analog reading
   * @param[in] right Right tape sensor analog reading
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
      // Both are on the tape
      // Go straight
      steering_pid.error = 0.0f;
    }
  }

  /**
   * @brief Processing the PID output and determining how the servo needs to move in response
   * @param[in] output PID output
   * @return Turning direction and PID output in string form
   * 
   * @note 90 is forwards, anything less is a right turn, vice versa for left turn
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
