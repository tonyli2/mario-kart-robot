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

    // Read the current value (Low on Tape, High elsewhere)\
    // If in IR tracking mode, use tape sensor variables (not marker sensor) to store IR readings
    if(pidType->isIR && FFT::hasFoundBeacon(IR_DETECTOR_LEFT, IR_DETECTOR_RIGHT, &(pidType->leftTapeInput), 
                    &(pidType->rightTapeInput), pidType->leftFFTHandler, pidType->rightFFTHandler)) {  
          
      digitalWrite(TEST_PIN_LED, HIGH);
    }
    else if(!pidType->isIR){
      // digitalWrite(TEST_PIN_LED, LOW);
      // Serial2.println(" TAPE TAPE TAPE TAPE ");
      pidType->leftTapeInput    = analogRead(LEFT_TAPE_PIN);
      pidType->leftMarkerInput  = analogRead(MARKER_SENSE_LEFT);
      pidType->rightTapeInput   = analogRead(RIGHT_TAPE_PIN);
      pidType->rightMarkerInput = analogRead(MARKER_SENSE_RIGHT);

      Serial2.print("LT: ");
      Serial2.print(pidType->leftTapeInput);
      Serial2.print("LM: ");
      Serial2.print(pidType->leftMarkerInput);
      Serial2.print("RT: ");
      Serial2.println(pidType->rightTapeInput);
      Serial2.print("RM: ");
      Serial2.print(pidType->rightMarkerInput);
    

    }

    // Calculate the error term
    calcError(pidType, &applyDifferential);

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
    processOutput(pidType, applyDifferential, servo);
  }

  /**
   * @brief Helper function to calculate the error via ADC
   * @param[in] left Left tape sensor analog reading
   * @param[in] right Right tape sensor analog reading
   */
  static void calcError(PID *pidType, bool *applyDifferential) {
    
    if (!pidType->isIR) {   // Tape tracking mode
        /*
        -1: Left is off tape
        0: both on tape
        1: right is off tape
        2: both off tape
      */

      if(pidType->justEscapedIR){
        pidType->error = 1.5f;
        *applyDifferential = false;
      }
      else if(pidType->leftMarkerInput > pidType->L_THRESHOLD && pidType->leftTapeInput > pidType->L_THRESHOLD
              && pidType->rightTapeInput > pidType->R_THRESHOLD && pidType->rightMarkerInput > pidType->R_THRESHOLD) {
        // All four sensors are off the tape
        // Look at previous error state and magnify it
        pidType->error = pidType->prevError * 2.0f;
        *applyDifferential = true;

      }
      else if(pidType->leftMarkerInput > pidType->L_THRESHOLD && pidType->leftTapeInput > pidType->L_THRESHOLD
              && pidType->rightTapeInput > pidType->R_THRESHOLD && pidType->rightMarkerInput < pidType->R_THRESHOLD) {
        // Left marker, left tape, right tape are off; right marker is on
        // TURN RIGHT by a big amount
        pidType->error = -1.0f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      } 
      else if(pidType->leftMarkerInput > pidType->L_THRESHOLD && pidType->leftTapeInput > pidType->L_THRESHOLD
              && pidType->rightTapeInput < pidType->R_THRESHOLD && pidType->rightMarkerInput < pidType->R_THRESHOLD) {
        // Left marker and left tape are off; right tape and marker are on
        // TURN RIGHT by a small amount
        pidType->error = -0.5f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      } 
      else if(pidType->leftMarkerInput < pidType->L_THRESHOLD && pidType->leftTapeInput < pidType->L_THRESHOLD
              && pidType->rightTapeInput > pidType->R_THRESHOLD && pidType->rightMarkerInput > pidType->R_THRESHOLD) {
        // Left marker and left tape are on; right tape and marker are off
        // TURN LEFT by a small amount
        pidType->error = 0.5f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      }
      else if(pidType->leftMarkerInput < pidType->L_THRESHOLD && pidType->leftTapeInput > pidType->L_THRESHOLD
              && pidType->rightTapeInput > pidType->R_THRESHOLD && pidType->rightMarkerInput > pidType->R_THRESHOLD) {
        // Left marker is on; left tape, right tape and right marker are off
        // TURN LEFT by a big amount
        pidType->error = 1.0f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      }
      else {
        // All four on the tape
        // Go straight
        pidType->error = 0.0f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      }

    } 
    
    else { // IR tracking mode

      // float_t averageAmplitude = (*left + *right) * 0.5f;

      //Dynamically change IR Threshold
      //-0.568f * averageAmplitude + 8300.0f;

      // if(pidType->IR_THRESHOLD < 0){
      //   pidType->IR_THRESHOLD = 0;
      // }

      if (pidType->leftTapeInput - pidType->rightTapeInput > pidType->IR_THRESHOLD) {
        // TURN left
        pidType->error = 1.0f;
        Serial2.println("TURN LEFT!!! ");
      } else if (pidType->rightTapeInput - pidType->leftTapeInput > pidType->IR_THRESHOLD) {
        // TURN right
        pidType->error = -1.0f;
        Serial2.println("TURN RIGHT!!! ");
      } else if (abs(pidType->leftTapeInput - pidType->rightTapeInput) < pidType->IR_THRESHOLD) {
        // GO STRAIGHT
        pidType->error = 0.0f;
        Serial2.println("STRAIGHT!!! ");
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
  static void processOutput(PID *pidType, bool applyDifferential, Servo servo) {

    //Limits output angle
    if(pidType->STRAIGHT_ANGLE + pidType->output > pidType->MAX_ANGLE) {
      pidType->output = pidType->MAX_ANGLE - pidType->STRAIGHT_ANGLE;
    } else if(pidType->STRAIGHT_ANGLE + pidType->output < pidType->MIN_ANGLE) {
      pidType->output = pidType->MIN_ANGLE - pidType->STRAIGHT_ANGLE;
    }

    // Process servo angle from PID output
    if(pidType->output < 0) {

      // duty_cycle = 20;
      if(!applyDifferential){
        DriverMotors::startMotorsForwardRight(pidType->STRAIGHT_SPEED);
        DriverMotors::startMotorsForwardLeft(pidType->STRAIGHT_SPEED);
      }
      else { //Apply differential in the back
        DriverMotors::startMotorsForwardRight(20);
        DriverMotors::startMotorsForwardLeft(45);
        // DriverMotors::startMotorsForwardRight(pidType->TURNING_SPEED * 0.6);
        // DriverMotors::startMotorsForwardLeft(pidType->TURNING_SPEED);
      }

      servo.write(pidType->STRAIGHT_ANGLE + pidType->output);

    } else if(pidType->output > 0) {

      // duty_cycle = 20;
      if(!applyDifferential){
        DriverMotors::startMotorsForwardRight(pidType->STRAIGHT_SPEED);
        DriverMotors::startMotorsForwardLeft(pidType->STRAIGHT_SPEED);
      }
      else { //Apply differential in the back
        DriverMotors::startMotorsForwardRight(45);
        DriverMotors::startMotorsForwardLeft(20);
        // DriverMotors::startMotorsForwardRight(pidType->TURNING_SPEED);
        // DriverMotors::startMotorsForwardLeft(pidType->TURNING_SPEED * 0.6);
      }
      servo.write(pidType->STRAIGHT_ANGLE + pidType->output);

    } else {
      // duty_cycle = 40;

      DriverMotors::startMotorsForwardRight(pidType->STRAIGHT_SPEED);
      DriverMotors::startMotorsForwardLeft(pidType->STRAIGHT_SPEED);


      servo.write(pidType->STRAIGHT_ANGLE);
    }
  }
}
