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
    if(!pidType->isIR) {
      pidType->leftInput = analogRead(LEFT_TAPE_PIN);
      pidType->rightInput = analogRead(RIGHT_TAPE_PIN);
    }
    else { // Otherwise, follow IR beacon, not tape

      // Check to see if 1kHz is found
      pidType->leftInput = FFT::runFFT(IR_DETECTOR_LEFT)[1];
      pidType->rightInput = FFT::runFFT(IR_DETECTOR_RIGHT)[1];

    }

    // Calculate the error term
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
    
    if (!pidType->isIR) {   // Tape tracking mode
        /*
        -1: Left is off tape
        0: both on tape
        1: right is off tape
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
    } else {              // IR tracking mode
      if (*left - *right > 100) {
        // Left IR signal is stronger
        // TURN left
        pidType->error = 1.0f;
      } else if (*right - *left > 100) {
        // Right IR signal is stronger
        // TURN right
        pidType->error = -1.0f;
      } else if (abs(*left - *right) < 100) {
        // Both IR signals are similar
        // GO STRAIGHT
        pidType->error = 0.0f;
      } else {
        // Left and right IR signals are both off
        // Look at previous error state and magnify it
        pidType->error = pidType->prevError * 2.0f;
      }
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

    u_int8_t duty_cycle = 0;

    //Limits output angle
    if(pidType->STRAIGHT_ANGLE + *output > pidType->MAX_ANGLE) {
      *output = pidType->MAX_ANGLE - pidType->STRAIGHT_ANGLE;
    } else if(pidType->STRAIGHT_ANGLE + *output < pidType->MIN_ANGLE) {
      *output = pidType->MIN_ANGLE - pidType->STRAIGHT_ANGLE;
    }

    // Process servo angle from PID output
    if(*output < 0) {

      // duty_cycle = 20;
      DriverMotors::startMotorsForwardRight(duty_cycle);
      DriverMotors::startMotorsForwardLeft(duty_cycle);

      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      pidType->servo.write(pidType->STRAIGHT_ANGLE + *output);
      return "Turn right (O: " + String(*output) + ")";

    } else if(*output > 0) {

      // duty_cycle = 20;
      DriverMotors::startMotorsForwardRight(duty_cycle);
      DriverMotors::startMotorsForwardLeft(duty_cycle);

      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      pidType->servo.write(pidType->STRAIGHT_ANGLE + *output);
      return "Turn left (O: " + String(*output) + ")";
    } else {
      // duty_cycle = 40;
      DriverMotors::startMotorsForwardRight(duty_cycle);
      DriverMotors::startMotorsForwardLeft(duty_cycle);

      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      pidType->servo.write(pidType->STRAIGHT_ANGLE);
      return "Go straight (O: " + String(*output) + ")";
    }
  }


}
