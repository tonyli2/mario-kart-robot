#include <config.h>

namespace Hivemind
{
    Servo servo;

    bool doneTurn = false;
    bool isGoingStraight = true;

    // Coasting timer
    uint32_t currentTime = 0;
    uint32_t prevTime = millis();
    uint32_t startIRTime = 0;
    uint32_t MAX_TIME = 1700;

    DigitalPID::PID steering_pid = {
        .Kp                 = 17.5f,
        .Ki                 = 0.0f,
        .Kd                 = 0.0f,
        .L_THRESHOLD        = 450.0f,
        .R_THRESHOLD        = 450.0f,
        .LM_THRESHOLD       = 450.0f,
        .RM_THRESHOLD       = 450.0f,
        .STRAIGHT_ANGLE     = 95,
        .MAX_INTEGRAL       = 100.0f,
        .leftTapeInput      = 0.0f,
        .leftMarkerInput    = 0.0f,
        .rightTapeInput     = 0.0f,
        .rightMarkerInput   = 0.0f,
        .error              = 0.0f,
        .prevError          = 0.0f,
        .derivative         = 0.0f,
        .integral           = 0.0f,
        .output             = 0.0f,
        .currTime           = 0,
        .prevTime           = 0,
        .t0                 = 0,
        .dt                 = 0,
        .MAX_ANGLE          = 117,
        .MIN_ANGLE          = 66,
        .isIR               = false,
        .TURNING_SPEED      = 0,
        .STRAIGHT_SPEED     = 65,
        .justEscapedIR      = false,
    };

    DigitalPID::PID ir_pid = {
        .Kp                 = 10.0f,
        .Ki                 = 0.0f,
        .Kd                 = 4000.0f,
        .IR_THRESHOLD       = 1500.0f,
        .STRAIGHT_ANGLE     = 95,
        .MAX_INTEGRAL       = 100.0f,
        .leftTapeInput      = 0.0f,
        .leftMarkerInput    = 0.0f,
        .rightTapeInput     = 0.0f,
        .rightMarkerInput   = 0.0f,
        .error              = 0.0f,
        .prevError          = 0.0f,
        .derivative         = 0.0f,
        .integral           = 0.0f,
        .output             = 0.0f,
        .currTime           = 0,
        .prevTime           = 0,
        .dt                 = 0,
        .MAX_ANGLE          = 117,
        .MIN_ANGLE          = 66,
        .isIR               = true,
        .TURNING_SPEED      = 40,
        .STRAIGHT_SPEED     = 55,
        .justEscapedIR      = false
    };

    robotState STATE = START;

    void wakeUpHivemind() {

        switch (STATE)
        {
        case START:
        {
            if(doneTurn) {
                // Once we are out of start sequence, move to IR detection
                prevTime = millis();
                startIRTime = millis();
                STATE = IR;
            }
            else {

                // Start position 1 - Left turn off of start
                if(digitalRead(START_POSITION) == HIGH) {
                    JumpHandler::turningSequence(&doneTurn, &isGoingStraight, 45.0f);
                }
                else { // Start Position 2 - Right turn off of start
                    JumpHandler::turningSequence(&doneTurn, &isGoingStraight, -45.0f);
                }
            }
        }
            break;
        
        case IR:
        {
            currentTime = millis();

            // Safety: if we look for IR for more than set time, we force state to change
            if(currentTime - startIRTime > MAX_TIME) {
                prevTime = currentTime;
                STATE = COAST;
            }
            else if(canExitIRFollowing()) {
                // Or if we CAN exit IR following, switch to Coasting
                prevTime = currentTime;
                STATE = COAST;
            }
                
            else {
                // Follow IR with PID tuning
                DigitalPID::applyPID(&ir_pid, servo);
            }
        
            break;

        }
        
        case COAST:
        {
            currentTime = millis();

            /*
                Redundancy: we will only start looking for tape after a
                set time to avoid picking up "fake" tapes. We will also only consider
                finding tape a positive if our IMU says we are on the ramp.

            */ 
            if(currentTime - prevTime > MAX_TIME && JumpHandler::hasFoundFourTape(false)
                && SensorFusion::isOnRamp) {
                
                // If we find the ramp-tape, transition to Tape-following
                prevTime = millis();
                STATE = TAPE;
                
            }
            else{
                Hivemind::setServo(steering_pid.MAX_ANGLE);
                DriverMotors::diffSteering(25, 75, true);
            }        
        }
            break;

        case TAPE:
        {
            currentTime = millis();
            
            if(currentTime - prevTime > MAX_TIME && JumpHandler::hasFoundFourTape(true)) {

                // If we find marker on ramp, trigger jump sequence 
                doneTurn = false; 
                isGoingStraight = false;
                prevTime = millis();
                SensorFusion::resetYaw();

                STATE = JUMP;
                
            }
            else {
                DigitalPID::applyPID(&steering_pid, servo);
            }

        }
            break;

        case JUMP:
        {
            if(!doneTurn) {
                JumpHandler::jumpHandler(&doneTurn, &isGoingStraight);
            }
            else{
                STATE = IR;
            }

        }
            break;

        default:
        {
            STATE = TAPE;
        }
            break;
        }

    }

    /**
     * @brief Setup function to initialize all functions
     * objects and other necessary components for the robot
     * in one single function call 
     */
    void setupHivemind() {

        Serial2.begin(9600);

        pinMode(TEST_PIN_LED, OUTPUT);
        pinMode(START_POSITION, INPUT_PULLDOWN); 
        pinMode(COLLISION_PIN, OUTPUT);
        pinMode(JUMP_PIN, OUTPUT);
        pinMode(START_LAP, INPUT_PULLDOWN);
        pinMode(PC13, OUTPUT);

        DigitalPID::setupServo(servo);
        SensorFusion::IMUInit();
       
    }

    /**
     * @brief Set the Servo object to specified angle
     * 
     * @param angle the desired angle
     */
    void setServo(uint8_t angle) {
        servo.write(angle);
    }

    /**
     * @brief Determines if we have gotten sufficiently close to the IR beacon.
     * This is determined by looking at the magnitudes of both the left and right
     * IR detectors mounted at the front of the robot.
     * 
     * @return true, we have gotten close enough to beacon
     * @return false, we are not close enough to beacon
     */
    bool canExitIRFollowing () {

        double_t escapeThreshold = 10600;

        if(ir_pid.leftTapeInput > escapeThreshold && 
            ir_pid.rightTapeInput > escapeThreshold){

            steering_pid.justEscapedIR = true;
            
        }
        else{
            steering_pid.justEscapedIR = false;            
        }

        return steering_pid.justEscapedIR;
    }

} // namespace Hivemind

