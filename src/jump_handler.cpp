#include <config.h>

namespace JumpHandler {

    float_t leftMarker = 0.0f;
    float_t rightMarker = 0.0f;
    float_t leftTapeSens = 0.0f;
    float_t rightTapeSens = 0.0f;
    const float_t LM_THRESHOLD = 450.0f;
    const float_t RM_THRESHOLD = 450.0f;

    const float_t LTape_THRESHOLD = 450.0f;
    const float_t RTape_THRESHOLD = 450.0f;

    const uint8_t sizeOfPitchArray = 10;
    float_t pitchArray[sizeOfPitchArray] = {0};
    uint8_t pitchCounter = 0;

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

        isGoingUpRamp = SensorFusion::isGoingUpRamp(pitchArray, pitchCounter, sizeOfPitchArray);

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
        attachInterrupt(JUMP_PIN, jumpHandler, RISING);
    }

    /**
     * @brief interrupt handler which processes logic 
     * required before and after bridge jump
     * 
     */
    void jumpHandler() {
        
        // TODO code for managing bridge jump here
        DriverMotors::stopMotorsBoth();

        // 1. Go Straight off bridge
        // DriverMotors::startMotorsForward(30);

        // 2. Trigger IMU sequence 

        // 3. Reset Interrupt Pin
        digitalWrite(JUMP_PIN, LOW);
    }

    void turningSequence(bool *doneTurn, bool *goStraight, float_t desiredAngle) {
        
        uint32_t turnSequenceSpeed = 70;

        if(*goStraight == true) {
            DriverMotors::startMotorsForwardLeft(turnSequenceSpeed);
            DriverMotors::startMotorsForwardRight(turnSequenceSpeed);
            delay(850);
            *goStraight = false;
        }

        float_t * imuData = SensorFusion::IMUGetData();

        // Serial2.print("Roll ");
        // Serial2.print(imuData[0]);
        // Serial2.print("    Pitch ");
        // Serial2.print(imuData[1]);
        // Serial2.print("    Yaw ");
        // Serial2.println(imuData[2]);

        if(desiredAngle >= 0) {
            if (imuData[2] <= calcIMUSteering(turnSequenceSpeed, desiredAngle)) {
                Hivemind::testServo(117);
                DriverMotors::diffSteeringLeft();
                *doneTurn = false;
            }
            else {
                Hivemind::testServo(95);
                DriverMotors::startMotorsForwardLeft(35);
                DriverMotors::startMotorsForwardRight(35);
                *doneTurn = true;
            }
        }
        else {
            if (imuData[2] >= calcIMUSteering(turnSequenceSpeed, desiredAngle)) {
                Hivemind::testServo(66);
                DriverMotors::diffSteeringRight();
                *doneTurn = false;
            }
            else {
                Hivemind::testServo(95);
                DriverMotors::startMotorsForwardLeft(35);
                DriverMotors::startMotorsForwardRight(35);
                *doneTurn = true;
            }
        }
    }

    float_t calcIMUSteering(uint32_t speed, float_t desiredAngle) {
        if(desiredAngle >= 0) {
            return desiredAngle - (0.2506f * speed - 14.176f) - 15.0f;
        }

        else {
            return desiredAngle + (0.2506f * speed - 14.176f) + 15.0f;
        }
        
    }
}
