#include <config.h>

namespace Hivemind
{
    Servo servo;

    // Testing
    bool doneTurn = false;
    bool isLapOne = true;
    bool isGoingStraight = true;

    // Coasting timer
    uint32_t currentTime = 0;
    uint32_t prevTime = millis();
    uint32_t startIRTime = 0;

    //TODO move this to a suitable location
    DigitalPID::PID steering_pid = {
        .Kp                 = 15.0f,
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
        .STRAIGHT_SPEED     = 45,
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
        .STRAIGHT_SPEED     = 55, // 55
        .justEscapedIR      = false
    };

    robotState STATE = TAPE;

    void wakeUpHivemind() {

        switch (STATE)
        {
        case START:
        {
            digitalWrite(TEST_PIN_LED, LOW);
            if(doneTurn) {
                prevTime = millis();
                startIRTime = millis();
                STATE = IR;
                //TODO add find IR escape condition
            }
            else {
                // if(digitalRead(START_POSITION) == HIGH) {
                    // Start Position 1
                    JumpHandler::turningSequence(&doneTurn, &isGoingStraight, 45.0f);
                // }
                // else {
                //     // Start Position 2
                //     DriverMotors::stopMotorsBoth();
                //     //TODO if have time, hard code turn without IMU
                //     // JumpHandler::turningSequence(&doneTurn, &isGoingStraight, -45.0f);
                // }
            }
        }
            break;
        
        case IR:
        {
            digitalWrite(TEST_PIN_LED, LOW);
            currentTime = millis();

            if(currentTime - startIRTime > 1600) {
                prevTime = millis();
                STATE = COAST;
            }

            // else if(currentTime - prevTime > 100) {
            //     // digitalWrite(TEST_PIN_LED, HIGH);
            //     if(canExitIRFollowing()) {
            //         // Hivemind::testServo(117);
            //         // DriverMotors::stopMotorsBoth();
            //         // delay(50000);
            //         STATE = COAST;
            //     }
            //     prevTime = currentTime;
            // }
            else {
                DigitalPID::applyPID(&ir_pid, servo);
            }
        
            break;

        }
        
        case COAST:
        {
            currentTime = millis();
            digitalWrite(TEST_PIN_LED, LOW);
            // If any of the avg pitch angles say we are not on the ramp, we stay coasting
            // for(int i = 0 ; i < 3; i++) {
            if(currentTime - prevTime > 1500) {
                if(JumpHandler::hasFoundFourTape(false)) {
                    digitalWrite(TEST_PIN_LED, LOW);
                    // Hivemind::testServo(95);
                    // DriverMotors::stopMotorsBoth();
                    // delay(50000);
                    prevTime = millis();
                    STATE = TAPE;
                }
                else{
                    Hivemind::testServo(117);
                    DriverMotors::upRampDiffLeft();
                }
            }
            // }

            // if(JumpHandler::hasFoundTape(false) && isGoingUpRamp) {
            // if(isGoingUpRamp) {
            //     // If you are not in Jump mode and find tape, switch to TAPE
            //     STATE = TAPE;
            // }
            else {
                Hivemind::testServo(117);
                DriverMotors::iRDiffLeft();
            }
        
        }
            break;

        case TAPE:
        {
            // currentTime = millis();

            // digitalWrite(TEST_PIN_LED, LOW);
            // // Hivemind::testServo(90);
            // // DriverMotors::stopMotorsBoth();
            // // delay(50000);
            // // DriverMotors::stopMotorsBoth();
            // // Hivemind::testServo(95);
            // // delay(20000);
            // if(currentTime - prevTime > 1500) {
            //     if(JumpHandler::hasFoundFourTape(true)) {

            //         digitalWrite(TEST_PIN_LED, HIGH);
            //         testServo(95);
            //         DriverMotors::stopMotorsBoth();
            //         delay(50000);
            //         doneTurn = false;
            //         prevTime = millis();
            //         STATE = JUMP;
            //     }
            // }
            // else {
            //     DigitalPID::applyPID(&steering_pid, servo);
            // }
            DigitalPID::applyPID(&steering_pid, servo);

        }
            break;

        case JUMP:
        {
            digitalWrite(TEST_PIN_LED, LOW);
            if(!doneTurn) {
                JumpHandler::jumpHandler(&doneTurn, &isGoingStraight);
            }
            else{
                STATE = IR;
            }

        }
            break;

        default:{}
            break;
        }

        // // Treat lap 1 special
        // if (!doneTurn && digitalRead(START_POSITION) == HIGH) {
        //     JumpHandler::turningSequence(&doneTurn, &isGoingStraight, 45.0f);
        //     // isLapOne = false;
        // }
        // else if(!doneTurn && digitalRead(START_POSITION) == LOW) {
        //     JumpHandler::turningSequence(&doneTurn, &isGoingStraight, -45.0f);
        // }
        // if(doneTurn && !canExitIRFollowing()) {
        //     digitalWrite(TEST_PIN_LED, LOW);
        //     DigitalPID::applyPID(&ir_pid, servo);
        // }
        // else if(doneTurn && canExitIRFollowing()) {
        //     if(!steering_pid.justEscapedIR) {
        //         digitalWrite(TEST_PIN_LED, HIGH);
        //     }
        //     DigitalPID::applyPID(&steering_pid, servo);
        // }
        // else if(isLapOne && START_POSITION == LOW) {
        //     JumpHandler::turningSequence(doneTurn, false, -90.0f);
        //     isLapOne = false;
        // }
        

        // if(JumpHandler::isReadyToJump()){
        //     //Trigger Jump Handler Interrupt
        //     digitalWrite(INTERRUPT_PIN, HIGH);

        //     //Once Jump Handler finished, find IR
        //     DigitalPID::applyPID(&ir_pid, servo);
        // }
        // else{
        //     DigitalPID::applyPID(&steering_pid, servo);
        // }
        // DigitalPID::applyPID(&steering_pid, servo);

        // Serial2.print(" Roll ");
        // Serial2.print(attitude_vec[0]);

        // Serial2.print("     Pitch ");
        // Serial2.print(attitude_vec[1]);

        // Serial2.print("     Yaw ");
        // Serial2.println(attitude_vec[2]);
        
        // if(isLapOne){
        //     testMotors();
        //     isLapOne = false;
        // }
        // float_t *attitude_vec = SensorFusion::IMUGetData();

        // if(attitude_vec[2] < 160){
        //     testServo(117);
        //     DriverMotors::diffSteeringLeft();
        // }
        // else{
        //     testServo(95);
        //     DriverMotors::stopMotorsBoth();
        // }

        // SensorFusion::IMUGetData();
        // SensorFusion::stopCar();

        // DigitalPID::applyPID(&steering_pid, servo);


        // Loops 
        // if(!doneTurn) {
        //     JumpHandler::afterJump(&doneTurn, &isGoingStraight);
        // }
        // else if(doneTurn && !canExitIRFollowing()) {
        //     DigitalPID::applyPID(&ir_pid, servo);
        // }
        // else if(doneTurn && canExitIRFollowing()) {
        //     DigitalPID::applyPID(&steering_pid, servo);
        // }


        // else {

        //     if(!JumpHandler::isReadyToJump()) {
        //         DigitalPID::applyPID(&steering_pid, servo);
        //     }
        //     else{
        //         digitalWrite(INTERRUPT_PIN, HIGH);
        //     }
        // }
        // if(steering_pid.justEscapedIR == true){
        //     digitalWrite(TEST_PIN_LED, HIGH);
        // }
        // else{
        //     digitalWrite(TEST_PIN_LED, LOW);
        // }

    }

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
        JumpHandler::setupJumpHandler();
        // int8_t leftOrRight = digitalRead(START_POSITION);
        // if (leftOrRight == LOW) {
        //     leftStartPos = true;
        //     // Serial2.println("Starting left");
        // } else {
        //     leftOrRight = false;
        //     // Serial2.println("Starting right");
        // }
    }

    void testServo(uint8_t angle) {
        servo.write(angle);
        // for(int i = 66; i < 118; i++){
        //     servo.write(i);
        //     Serial2.println(i);
        //     delay(15);
        // }
        // for(int i = 117; i > 65; i--){
        //     servo.write(i);

        //     delay(15);
        // }

    }

    void testMotors(){

        DriverMotors::startMotorsForwardLeft(90);
        DriverMotors::startMotorsForwardRight(90);
        delay(1500);
        // DriverMotors::stopMotorsBoth();

        // delay(10000);      
    }

    bool canExitIRFollowing () {

        double_t escapeThreshold = 10600;

        // Serial2.print(" L ");
        // Serial2.print(ir_pid.leftTapeInput);
        // Serial2.print(" R ");
        // Serial2.println(ir_pid.rightTapeInput);

        if(ir_pid.leftTapeInput > escapeThreshold && 
            ir_pid.rightTapeInput > escapeThreshold){

            steering_pid.justEscapedIR = true;
            // return steering_pid.justEscapedIR;
            return true;
        }
        else{
            steering_pid.justEscapedIR = false;
            // return steering_pid.justEscapedIR;
            return false;
        }
    }

} // namespace Hivemind

