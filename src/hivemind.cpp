#include <config.h>

namespace Hivemind
{
    //TODO move this to a suitable location
    DigitalPID::PID steering_pid = {
        .Kp                 = 15.0f,
        .Ki                 = 0.0f,
        .Kd                 = 0.0f,
        .L_THRESHOLD        = 390.0f,
        .R_THRESHOLD        = 390.0f,
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
        .Kp                 = 10.0f,
        .Ki                 = 0.0f,
        .Kd                 = 0.0f,
        .L_THRESHOLD        = 380.0f,
        .R_THRESHOLD        = 380.0f,
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

    String wakeUpHivemind() {
        // if(LocationIdentifier::isReadyToJump()){
        //     //Trigger Jump Handler Interrupt
        //     digitalWrite(INTERRUPT_PIN, HIGH);

        //     //Once Jump Handler finished, find IR
        //     DigitalPID::applyPID(&ir_pid);
        // }
        // else{
        //     DigitalPID::applyPID(&steering_pid);
        // }

        return DigitalPID::applyPID(&steering_pid);
    }

    String irHivemind(){

        return DigitalPID::applyPID(&ir_pid);
    }

    void setupHivemind(){
        DigitalPID::setupServo(&steering_pid);
    }

    void testServo(uint8_t angle){
        steering_pid.servo.write(angle);
        // for(int i = 73; i < 118; i++){
        //     steering_pid.servo.write(i);
        //     delay(15);
        // }
        // for(int i = 117; i > 72; i--){
        //     steering_pid.servo.write(i);
        //     delay(15);
        // }

    }

    void testMotors(){

        DriverMotors::startMotorsForwardLeft(30);
        // DriverMotors::startMotorsForwardRight(30);

        delay(1000);

        DriverMotors::stopMotorsLeft();
        // DriverMotors::stopMotorsRight();

        delay(1000);
        
        DriverMotors::startMotorsBackwardLeft(30);
        // // DriverMotors::startMotorsBackwardRight(30);

        delay(1000);

        DriverMotors::stopMotorsLeft();
        // // DriverMotors::stopMotorsRight();

        
    }

    void testIR() {

        if(FFT::hasFoundBeacon(IR_DETECTOR_LEFT, IR_DETECTOR_RIGHT)){
            ir_pid.leftInput = FFT::runFFT(IR_DETECTOR_LEFT)[1];
            ir_pid.rightInput = FFT::runFFT(IR_DETECTOR_RIGHT)[1];
        }

        if(ir_pid.leftInput < ir_pid.rightInput){
            steering_pid.servo.write(80);
            delay(2000);
        }
        else{
            steering_pid.servo.write(110);
            delay(2000);
        }

    }
} // namespace Hivemind

