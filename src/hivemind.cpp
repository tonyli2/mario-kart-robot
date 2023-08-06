#include <config.h>

namespace Hivemind
{
    Servo servo;

    // Testing
    bool doneTurn = false;
    bool isLapOne = true;

    //TODO move this to a suitable location
    DigitalPID::PID steering_pid = {
        .Kp                 = 15.0f,
        .Ki                 = 0.0f,
        .Kd                 = 0.0f,
        .L_THRESHOLD        = 500.0f,
        .R_THRESHOLD        = 500.0f,
        .STRAIGHT_ANGLE     = 95,
        .MAX_INTEGRAL       = 100.0f,
        .leftInput          = 0.0f,
        .rightInput         = 0.0f,
        .error              = 0.0f,
        .prevError          = 0.0f,
        .derivative         = 0.0f,
        .integral           = 0.0f,
        .output             = 0.0f,
        .currTime           = 0,
        .prevTime           = 0,
        .dt                 = 0,
        .MAX_ANGLE          = 117,
        .MIN_ANGLE          = 73,
        .isIR               = false,
        .TURNING_SPEED      = 55,
        .STRAIGHT_SPEED     = 30,
        .justEscapedIR      = false,
    };

    DigitalPID::PID ir_pid = {
        .Kp                 = 15.0f,
        .Ki                 = 0.0f,
        .Kd                 = 12.5f,
        .IR_THRESHOLD       = 1500.0f,
        .STRAIGHT_ANGLE     = 95,
        .MAX_INTEGRAL       = 100.0f,
        .leftInput          = 0.0f,
        .rightInput         = 0.0f,
        .error              = 0.0f,
        .prevError          = 0.0f,
        .derivative         = 0.0f,
        .integral           = 0.0f,
        .output             = 0.0f,
        .currTime           = 0,
        .prevTime           = 0,
        .dt                 = 0,
        .MAX_ANGLE          = 117,
        .MIN_ANGLE          = 73,
        .isIR               = true,
        .TURNING_SPEED      = 40,
        .STRAIGHT_SPEED     = 55,
        .justEscapedIR      = false
    };

    void wakeUpHivemind() {

        // Treat lap 1 special
        // if (isLapOne) {
        //     float_t *attitude_vec = SensorFusion::IMUGetData();
            
        //     if (abs(attitude_vec[2]) <= 90) {
        //         // If starting at left (pos 1), immediately turn left
        //         if (leftStartPos) {
        //             servo.write(117);
        //             DriverMotors::diffSteeringLeft();
        //         }
        //         // If starting at right (pos 2), immediately turn right
        //         else {
        //             servo.write(73);
        //             DriverMotors::diffSteeringRight();
        //         }
        //         return;
        //     }

        //     DigitalPID::applyPID(&ir_pid, servo);
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

        Hivemind::testMotors();
        SensorFusion::IMUGetData();
        SensorFusion::stopCar();

        // if(!doneTurn) {
        //     JumpHandler::afterJump(&doneTurn);
        // }
        // else if(doneTurn && !canExitIRFollowing()) {
        //     DigitalPID::applyPID(&ir_pid, servo);
        // }
        // else if(doneTurn && canExitIRFollowing()) {
        //     // DigitalPID::applyPID(&steering_pid, servo);
        //     Hivemind::testServo(steering_pid.STRAIGHT_ANGLE);
        //     DriverMotors::stopMotorsBoth();
        // }


        // else {

        //     if(!JumpHandler::isReadyToJump()) {
        //         DigitalPID::applyPID(&steering_pid, servo);
        //     }
        //     else{
        //         digitalWrite(INTERRUPT_PIN, HIGH);
        //     }
        // }
        if(steering_pid.justEscapedIR == true){
            digitalWrite(TEST_PIN_LED, HIGH);
        }
        else{
            digitalWrite(TEST_PIN_LED, LOW);
        }

    }

    void setupHivemind() {

        Serial2.begin(9600);

        pinMode(TEST_PIN_LED, OUTPUT);
        pinMode(START_POSITION, INPUT_PULLDOWN); 
        pinMode(COLLISION_PIN, OUTPUT);
        pinMode(JUMP_PIN, OUTPUT);

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

    void testServo(uint8_t angle){
        servo.write(angle);
        // for(int i = 73; i < 118; i++){
        //     servo.write(i);
        //     delay(15);
        // }
        // for(int i = 117; i > 72; i--){
        //     servo.write(i);
        //     delay(15);
        // }

    }

    void testMotors(){

        DriverMotors::startMotorsForwardLeft(30);
        DriverMotors::startMotorsForwardRight(30);

        // DriverMotors::stopMotorsBoth();

        // delay(10000);      
    }

    bool canExitIRFollowing () {

        double_t escapeThreshold = 13600;

        if(ir_pid.leftInput > escapeThreshold && 
            ir_pid.rightInput > escapeThreshold){

            steering_pid.justEscapedIR = true;
            return steering_pid.justEscapedIR;
        }
        else{
            steering_pid.justEscapedIR = false;
            return steering_pid.justEscapedIR;
        }
    }

} // namespace Hivemind

