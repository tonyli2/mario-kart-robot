#include <config.h>

namespace DigitalPID {
  
  /**
   * @brief Attaching servo to the specified pin
   * 
   */
  void setupServo(Servo servo) {
    servo.attach(STEERING_SERVO);
  }

  /**
   * @brief Applying PID calculations and converting sensor inputs to servo angles
   * @return Turning direction and PID output in string form
   * 
   * @note Might change return type to void once finishing PID tunning
   */
  void applyPID(PID *pidType, Servo servo) {

    bool applyDifferential = false;

    // Calculate the elapsed time since the last iteration
    pidType->currTime = millis();
    pidType->dt = pidType->currTime - pidType->prevTime;

    // Read the current value (Low on Tape, High elsewhere)
    if(pidType->isIR && FFT::hasFoundBeacon(IR_DETECTOR_LEFT, IR_DETECTOR_RIGHT, &(pidType->leftInput), 
                    &(pidType->rightInput), pidType->leftFFTHandler, pidType->rightFFTHandler)) {  
          
      digitalWrite(TEST_PIN_LED, HIGH);
    }
    else {
      digitalWrite(TEST_PIN_LED, LOW);
      Serial2.print(" ____________________________________________ ");
      pidType->leftInput = analogRead(LEFT_TAPE_PIN);
      pidType->rightInput = analogRead(RIGHT_TAPE_PIN);
    }

    // Calculate the error term
    calcError(&(pidType->leftInput), &(pidType->rightInput), pidType);

    if(pidType->error == pidType->prevError * 2.0){
      applyDifferential = true;
    }

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
    pidType->output = pidType->Kp * pidType->error + pidType->Ki * pidType->integral 
                      + pidType->Kd * pidType->derivative;

    // Update the previous error and time for the next iteration
    pidType->prevError = pidType->error;
    pidType->prevTime = pidType->currTime;

    // Apply the output (e.g. turn servo)
    processOutput(&(pidType->output), pidType, applyDifferential, servo);
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

    } else {
      // IR tracking mode
      // float_t averageAmplitude = (*left + *right) * 0.5f;

      //Dynamically change IR Threshold
      pidType->IR_THRESHOLD = 3000;//-0.568f * averageAmplitude + 8300.0f;

      if(pidType->IR_THRESHOLD < 0){
        pidType->IR_THRESHOLD = 0;
      }

      if (*left - *right > pidType->IR_THRESHOLD) {
        // TURN left
        pidType->error = 1.0f;
        Serial2.print("TURN LEFT!!! ");
      } else if (*right - *left > pidType->IR_THRESHOLD) {
        // TURN right
        pidType->error = -1.0f;
        Serial2.print("TURN RIGHT!!! ");
      } else if (abs(*left - *right) < pidType->IR_THRESHOLD) {
        // GO STRAIGHT
        pidType->error = 0.0f;
        Serial2.print("STRAIGHT!!! ");
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
  static void processOutput(float_t *output, PID *pidType, 
                              bool applyDifferential, Servo servo) {

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
      if(!applyDifferential){
        DriverMotors::startMotorsForwardRight(duty_cycle);
        DriverMotors::startMotorsForwardLeft(duty_cycle);
      }
      else { //Apply differential in the back
        DriverMotors::startMotorsForwardRight(duty_cycle - duty_cycle * 0.75);
        // DriverMotors::startMotorsForwardLeft(55);
      }

      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      servo.write(pidType->STRAIGHT_ANGLE + *output);

    } else if(*output > 0) {

      // duty_cycle = 20;
      if(!applyDifferential){
        DriverMotors::startMotorsForwardRight(duty_cycle);
        DriverMotors::startMotorsForwardLeft(duty_cycle);
      }
      else { //Apply differential in the back
        // DriverMotors::startMotorsForwardRight(55);
        DriverMotors::startMotorsForwardLeft(duty_cycle - duty_cycle * 0.75);
      }

      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      servo.write(pidType->STRAIGHT_ANGLE + *output);

    } else {
      // duty_cycle = 40;
      DriverMotors::startMotorsForwardRight(duty_cycle);
      DriverMotors::startMotorsForwardLeft(duty_cycle);

      // String duty_cycle_print = "Duty Cycle: " + String(duty_cycle);
      // display_handler.println(duty_cycle_print);
      servo.write(pidType->STRAIGHT_ANGLE);
    }
  }


}
