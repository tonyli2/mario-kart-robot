#include <config.h>

namespace Hivemind
{
    //TODO move this to a suitable location
    DigitalPID::PID steering_pid = {
        .Kp                 = 40.0f,
        .Ki                 = 0.0f,
        .Kd                 = 10.0f,
        .L_THRESHOLD        = 600.0f,
        .R_THRESHOLD        = 600.0f,
        .STRAIGHT_ANGLE     = 90,
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
        .MAX_ANGLE          = 50,
        .MIN_ANGLE          = -50,
    };

    DigitalPID::PID ir_pid = {
        .Kp                 = 40.0f,
        .Ki                 = 0.0f,
        .Kd                 = 0.0f,
        .L_THRESHOLD        = 600.0f,
        .R_THRESHOLD        = 600.0f,
        .STRAIGHT_ANGLE     = 90,
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
        .MAX_ANGLE          = 50,
        .MIN_ANGLE          = -50,
    };

    void wakeUpHivemind() {
        if(LocationIdentifier::isReadyToJump()){
            //Trigger Jump Handler Interrupt
            digitalWrite(INTERRUPT_PIN, HIGH);

            //Once Jump Handler finished, find IR
            // DigitalPID::PID *ir = &DigitalPID
            DigitalPID::applyPID(&ir_pid);
        }
        else{
            DigitalPID::applyPID(&steering_pid);
        }
    }
} // namespace Hivemind

