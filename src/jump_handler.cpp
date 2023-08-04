#include <config.h>

namespace JumpHandler {

    float_t leftMarker = 0.0f;
    float_t rightMarker = 0.0f;
    float_t leftTapeSens = 0.0f;
    float_t rightTapeSens = 0.0f;
    const float_t LM_THRESHOLD = 390.0f;
    const float_t RM_THRESHOLD = 390.0f;

    const float_t LTape_THRESHOLD = 390.0f;
    const float_t RTape_THRESHOLD = 390.0f;

    bool isGoingUpRamp = false;

    /**
     * @brief determines whether the robot has detected the tape marker that
     * indicates an impending jump, and whether it should trigger the
     * jump handler interrupt sequence.
     * 
     * @return true the robot is supposed to trigger the jump handler sequence
     * @return false the robot is not supposed to trigger the jump handler sequence
     */
    bool isReadyToJump(){
        
        leftMarker = analogRead(MARKER_SENSE_LEFT);
        rightMarker = analogRead(MARKER_SENSE_RIGHT);

        leftTapeSens = analogRead(LEFT_TAPE_PIN);
        rightTapeSens = analogRead(RIGHT_TAPE_PIN);

        isGoingUpRamp = SensorFusion::isGoingUpRamp();

        bool readyToJump = 
            leftMarker < LM_THRESHOLD && 
            rightMarker < RM_THRESHOLD &&
            leftTapeSens < LTape_THRESHOLD &&
            rightTapeSens < RTape_THRESHOLD &&
            isGoingUpRamp;

        return readyToJump;
    }

    /**
     * @brief sets up IRQ Handler for Marker Reader
     * reflectance sensors
     * 
     */
    void setupJumpHandler() {
        pinMode(INTERRUPT_PIN, OUTPUT);
        attachInterrupt(INTERRUPT_PIN, jumpHandler, RISING);
    }

    /**
     * @brief interrupt handler which processes logic 
     * required before and after bridge jump
     * 
     */
    void jumpHandler(){
        
        // TODO code for managing bridge jump here

        // 1. Go Straight off bridge
        // DriverMotors::startMotorsForward(30);

        // 2. Trigger IMU sequence (reaction wheel)
        
        // 3. When landed, use IMU to turn robot to face beacon

        // 4. Reset Interrupt Pin
        digitalWrite(INTERRUPT_PIN, LOW);
    }
}
