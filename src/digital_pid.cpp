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
    pidType->dt = pidType->currTime - pidType->t0;

    // Read the current value (Low on Tape, High elsewhere)
    // If in IR tracking mode, use tape sensor variables (not marker sensor) to store IR readings
    if(pidType->isIR && FFT::hasFoundBeacon(IR_DETECTOR_LEFT, IR_DETECTOR_RIGHT, &(pidType->leftTapeInput), 
                    &(pidType->rightTapeInput), pidType->leftFFTHandler, pidType->rightFFTHandler)) {  
        // hasFoundBeacon modifies left and right input
    }
    else if(!pidType->isIR){
      
      pidType->leftTapeInput    = analogRead(LEFT_TAPE_PIN);
      pidType->leftMarkerInput  = analogRead(MARKER_SENSE_LEFT);
      pidType->rightTapeInput   = analogRead(RIGHT_TAPE_PIN);
      pidType->rightMarkerInput = analogRead(MARKER_SENSE_RIGHT);
    
    }

    // Calculate the error term
    calcError(pidType, &applyDifferential);

    if (pidType->error != pidType->prevError) {
      pidType->t0 = millis();
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
      
      if(pidType->leftMarkerInput >= pidType->LM_THRESHOLD && pidType->leftTapeInput >= pidType->L_THRESHOLD
              && pidType->rightTapeInput >= pidType->R_THRESHOLD && pidType->rightMarkerInput >= pidType->RM_THRESHOLD) {
        // All four sensors are off the tape
        // Look at previous error state and magnify it

        if(pidType->prevError < 0){
          pidType->error = -2.0f;
          *applyDifferential = true;
        }
        else if(pidType->prevError > 0){
          pidType->error = 2.0f;
          *applyDifferential = true;
        }
        else{
          pidType->error = 0.0f;
          *applyDifferential = false;
        }

      }
      else if(pidType->leftMarkerInput >= pidType->LM_THRESHOLD && pidType->leftTapeInput >= pidType->L_THRESHOLD
              && pidType->rightTapeInput >= pidType->R_THRESHOLD && pidType->rightMarkerInput < pidType->RM_THRESHOLD) {
        // Left marker, left tape, right tape are off; right marker is on
        // TURN RIGHT by a big amount
        pidType->error = -1.0f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      } 
      else if(pidType->leftMarkerInput >= pidType->LM_THRESHOLD && pidType->leftTapeInput >= pidType->L_THRESHOLD
              && pidType->rightTapeInput < pidType->R_THRESHOLD && pidType->rightMarkerInput < pidType->RM_THRESHOLD) {
        // Left marker and left tape are off; right tape and marker are on
        // TURN RIGHT by a small amount
        pidType->error = -0.5f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      }
      else if(pidType->leftMarkerInput < pidType->LM_THRESHOLD && pidType->leftTapeInput < pidType->L_THRESHOLD
              && pidType->rightTapeInput >= pidType->R_THRESHOLD && pidType->rightMarkerInput >= pidType->RM_THRESHOLD) {
        // Left marker and left tape are on; right tape and marker are off
        // TURN LEFT by a small amount
        pidType->error = 0.5f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      }
      else if(pidType->leftMarkerInput < pidType->LM_THRESHOLD && pidType->leftTapeInput >= pidType->L_THRESHOLD
              && pidType->rightTapeInput >= pidType->R_THRESHOLD && pidType->rightMarkerInput >= pidType->RM_THRESHOLD) {
        // Left marker is on; left tape, right tape and right marker are off
        // TURN LEFT by a big amount
        pidType->error = 1.0f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;

      }
      else if(pidType->leftMarkerInput >= pidType->LM_THRESHOLD && pidType->leftTapeInput < pidType->L_THRESHOLD
              && pidType->rightTapeInput < pidType->R_THRESHOLD && pidType->rightMarkerInput >= pidType->RM_THRESHOLD){
        // Only middle two are on tape
        // Go straight
        pidType->error = 0.0f;
        *applyDifferential = false;
        pidType->justEscapedIR = false;
        
      }
      else if(pidType->leftMarkerInput < pidType->LM_THRESHOLD && pidType->leftTapeInput < pidType->L_THRESHOLD
              && pidType->rightTapeInput < pidType->R_THRESHOLD && pidType->rightMarkerInput < pidType->RM_THRESHOLD) {
        
        pidType->error = 0;
        *applyDifferential = false;
        pidType->justEscapedIR = false;
      }
      else{
        pidType->error = pidType->prevError;          
      }
    } 
    
    else { // IR tracking mode

      if (pidType->leftTapeInput - pidType->rightTapeInput > pidType->IR_THRESHOLD) {
        // TURN left
        pidType->error = 1.0f;
        
      } else if (pidType->rightTapeInput - pidType->leftTapeInput > pidType->IR_THRESHOLD) {
        // TURN right
        pidType->error = -1.0f;
        
      } else if (abs(pidType->leftTapeInput - pidType->rightTapeInput) < pidType->IR_THRESHOLD) {
        // GO STRAIGHT
        pidType->error = 0.0f;
        
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
    else if(pidType->output < 0) {

      if(!applyDifferential){
        DriverMotors::startMotorsForwardRight(pidType->STRAIGHT_SPEED);
        DriverMotors::startMotorsForwardLeft(pidType->STRAIGHT_SPEED);
      }
      else { //Apply differential for right turn 
        DriverMotors::diffSteering(70, 20, false);
      }

      servo.write(pidType->STRAIGHT_ANGLE + pidType->output);

    } else if(pidType->output > 0) {

      if(!applyDifferential){
        DriverMotors::startMotorsForwardRight(pidType->STRAIGHT_SPEED);
        DriverMotors::startMotorsForwardLeft(pidType->STRAIGHT_SPEED);
      }
      else { //Apply differential for left turn
        DriverMotors::diffSteering(20, 70, true);
      }
      servo.write(pidType->STRAIGHT_ANGLE + pidType->output);

    } else {
      DriverMotors::startMotorsForwardRight(pidType->STRAIGHT_SPEED);
      DriverMotors::startMotorsForwardLeft(pidType->STRAIGHT_SPEED);
      servo.write(pidType->STRAIGHT_ANGLE);
    }
  }
}
