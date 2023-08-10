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

    /**
     * @brief determines whether the robot has detected the tape marker that
     * indicates an impending jump, and whether it should trigger the
     * jump handler interrupt sequence.
     * 
     * @return true the robot is supposed to trigger the jump handler sequence
     * @return false the robot is not supposed to trigger the jump handler sequence
     */
    bool hasFoundFourTape(bool isJumping) {
        
        leftMarker = analogRead(MARKER_SENSE_LEFT);
        rightMarker = analogRead(MARKER_SENSE_RIGHT);

        leftTapeSens = analogRead(LEFT_TAPE_PIN);
        rightTapeSens = analogRead(RIGHT_TAPE_PIN);

        if(isJumping) {
            // Jumping mode
            bool readyToJump = 
                leftMarker < LM_THRESHOLD &&
                rightMarker < RM_THRESHOLD &&
                leftTapeSens < LTape_THRESHOLD &&
                rightTapeSens < RTape_THRESHOLD;

            return readyToJump;
        }
        else {
            // Coasting state
            bool seenTape = 
                leftMarker < LM_THRESHOLD || 
                rightMarker < RM_THRESHOLD ||
                leftTapeSens < LTape_THRESHOLD ||
                rightTapeSens < RTape_THRESHOLD;

            return seenTape;
        }
    }

    /**
     * @brief sets up IRQ Handler for Marker Reader
     * reflectance sensors
     * 
     */
    void setupJumpHandler() {

        //TODO remove
        // attachInterrupt(JUMP_PIN, jumpHandler, RISING);
    }

    /**
     * @brief interrupt handler which processes logic 
     * required before and after bridge jump
     * 
     */
    void jumpHandler(bool *doneTurn, bool *goStraight) {
        
        // TODO code for managing bridge jump here
        // DriverMotors::stopMotorsBoth();

        // 1. Reset IMU yaw angle
        SensorFusion::resetYaw();

        // 2. Go Straight off bridge
        DriverMotors::startMotorsForwardLeft(70);
        DriverMotors::startMotorsForwardRight(70);

        delay(500);
        // 3. Trigger IMU sequence
        turningSequence(doneTurn, goStraight, 180);
    }

    void turningSequence(bool *doneTurn, bool *goStraight, float_t desiredAngle) {
        
        uint32_t turnSequenceSpeed = 70;

        if(*goStraight == true) {
            Hivemind::testServo(90);
            DriverMotors::startMotorsForwardLeft(turnSequenceSpeed);
            DriverMotors::startMotorsForwardRight(turnSequenceSpeed);
            delay(900);
            *goStraight = false;
        }

        float_t * imuData = SensorFusion::IMUGetData();
        // Serial2.print("Yaw angle: ");
        // Serial2.println(imuData[2]);

        // Serial2.print("Roll ");
        // Serial2.print(imuData[0]);
        // Serial2.print("    Pitch ");
        // Serial2.print(imuData[1]);
        // Serial2.print("    Yaw ");
        // Serial2.println(imuData[2]);

        if(desiredAngle >= 0) {
            if (imuData[2] <= (calcIMUSteering(turnSequenceSpeed, desiredAngle) - 20)) {
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
        // TODO add back if ready for start position 2
        // else {
        //     if (imuData[2] >= calcIMUSteering(turnSequenceSpeed, desiredAngle)) {
        //         Hivemind::testServo(66);
        //         DriverMotors::diffSteeringRight();
        //         *doneTurn = false;
        //     }
        //     else {
        //         Hivemind::testServo(95);
        //         DriverMotors::startMotorsForwardLeft(35);
        //         DriverMotors::startMotorsForwardRight(35);
        //         *doneTurn = true;
        //     }
        // }
    }

    float_t calcIMUSteering(uint32_t speed, float_t desiredAngle) {
        if(desiredAngle >= 0) {
            return desiredAngle - (0.2506f * speed - 14.176f) - 20.0f;
        }

        else {
            return desiredAngle + (0.2506f * speed - 14.176f) + 20.0f;
        }
        
    }
}
