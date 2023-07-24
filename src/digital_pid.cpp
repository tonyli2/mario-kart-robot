#include <config.h>

namespace DigitalPID {
  
  /**
   * @brief Attaching servo to the specified pin
   * 
   */
  void setupServo(PID *pidType) {
    pidType->servo.attach(STEERING_SERVO);
  }

  /**
   * @brief Applying PID calculations and converting sensor inputs to servo angles
   * @return Turning direction and PID output in string form
   * 
   * @note Might change return type to void once finishing PID tunning
   */
  String applyPID(PID *pidType) {
    // Calculate the elapsed time since the last iteration
    pidType->currTime = millis();
    pidType->dt = pidType->currTime - pidType->prevTime;

    // Read the current value (Low on Tape, High elsewhere)
    pidType->leftInput = analogRead(LEFT_TAPE_PIN);
    pidType->rightInput = analogRead(RIGHT_TAPE_PIN);

    // Calculate the error term
    // TODO: interrupt with IR when not seeing tape
    calcError(&(pidType->leftInput), &(pidType->rightInput), pidType);

    // Calculate the integral term
    pidType->integral += pidType->error * pidType->dt;
    // Limit integral contribution in case we have constant error
    if(pidType->integral > pidType->MAX_INTEGRAL) {
      pidType->integral = pidType->MAX_INTEGRAL;
    } else if (pidType->integral < pidType->MIN_ANGLE) {
      pidType->integral = pidType->MIN_ANGLE;
    }

    // Calculate the derivative term
    pidType->derivative = (pidType->error - pidType->prevError) / pidType->dt;

    // Calculate the PID output
    pidType->output = pidType->Kp * pidType->error + pidType->Ki * pidType->integral + pidType->Kd * pidType->derivative;

    // Update the previous error and time for the next iteration
    pidType->prevError = pidType->error;
    pidType->prevTime = pidType->currTime;

    // Apply the output (e.g. turn servo)
    return processOutput(&(pidType->output), pidType);
  }

  /**
   * @brief Helper function to calculate the error via ADC
   * @param[in] left Left tape sensor analog reading
   * @param[in] right Right tape sensor analog reading
   */
  static void calcError(float_t *left, float_t *right, PID *pidType) {
    
    /*
      -1: Left is off tape
       0: both on tape
       1: right is on tape
       2: both off tape
    */

    if(*left > pidType->L_THRESHOLD && *right < pidType->R_THRESHOLD) {
      // Left is off tape and Right is on tape
      // TURN RIGHT
      pidType->error = -1.0f;
    } else if(*left < pidType->L_THRESHOLD && *right > pidType->R_THRESHOLD) {
      // Left is on tape and Right is off tape
      // TURN LEFT
      pidType->error = 1.0f;
    } else if(*left > pidType->L_THRESHOLD && *right > pidType->R_THRESHOLD) {
      // Left is off tape and Right is off tape
      // Look at previous error state and magnify it
      pidType->error = pidType->prevError * 2.0f;
    } else {
      // Both are on the tape
      // Go straight
      pidType->error = 0.0f;
    }
  }

  /**
   * @brief Processing the PID output and determining how the servo needs to move in response
   * @param[in] output PID output
   * @return Turning direction and PID output in string form
   * 
   * @note 90 is forwards, anything less is a right turn, vice versa for left turn
   */
  static String processOutput(float_t *output, PID *pidType) {

    int8_t duty_cycle = 15;

    //Limits output angle
    if(*output > pidType->MAX_ANGLE) {
      *output = pidType->MAX_ANGLE;
    } else if(*output < pidType->MIN_ANGLE) {
      *output = pidType->MIN_ANGLE;
    }

    // Process servo angle from PID output
    if(*output < 0) {
      duty_cycle = 15;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      pidType->servo.write(pidType->STRAIGHT_ANGLE - *output);
      return "Turn right (O: " + String(*output) + ")";
    } else if(*output > 0) {
      duty_cycle = 15;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      pidType->servo.write(pidType->STRAIGHT_ANGLE - *output);
      return "Turn left (O: " + String(*output) + ")";
    } else {
      duty_cycle = 40;
      DriverMotors::startMotorsForward(duty_cycle);
      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      pidType->servo.write(pidType->STRAIGHT_ANGLE + *output);
      return "Go straight (O: " + String(*output) + ")";
    }
  }
}
