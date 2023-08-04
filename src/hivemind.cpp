#include <config.h>

namespace Hivemind
{
    Servo servo;

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
        .isIR               = false
    };

    DigitalPID::PID ir_pid = {
        .Kp                 = 20.0f,
        .Ki                 = 0.0f,
        .Kd                 = 0.0f,
        .IR_THRESHOLD       = 0.0f,
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
        .isIR               = true
    };

    void wakeUpHivemind() {
        // if(JumpHandler::isReadyToJump()){
        //     //Trigger Jump Handler Interrupt
        //     digitalWrite(INTERRUPT_PIN, HIGH);

        //     //Once Jump Handler finished, find IR
        //     DigitalPID::applyPID(&ir_pid, servo);
        // }
        // else{
        //     DigitalPID::applyPID(&steering_pid, servo);
        // }

        DigitalPID::applyPID(&ir_pid, servo);
    }

    void setupHivemind(){
        Serial2.begin(9600);
        pinMode(TEST_PIN_LED, OUTPUT);
        DigitalPID::setupServo(servo);
    }

    void testServo(uint8_t angle){
        // steering_pid.servo.write(angle);
        for(int i = 73; i < 118; i++){
            servo.write(i);
            delay(15);
        }
        for(int i = 117; i > 72; i--){
            servo.write(i);
            delay(15);
        }

    }

    void testMotors(){

        DriverMotors::startMotorsForwardLeft(30);
        DriverMotors::startMotorsForwardRight(30);

        delay(2000);

        DriverMotors::stopMotorsLeft();
        DriverMotors::stopMotorsRight();

        delay(10000);
        
        // DriverMotors::startMotorsBackwardLeft(30);
        // DriverMotors::startMotorsBackwardRight(30);

        // delay(1000);

        // DriverMotors::stopMotorsLeft();
        // // DriverMotors::stopMotorsRight();

        
    }

} // namespace Hivemind

