#include <driver_motors.h>
#include <config.h>

namespace DriverMotors{

    /*
        @brief Drive the rear-wheel motors forward
        at specified PWM duty cycle
    */
    void startMotorsForward(short dutyCycle){
        pwm_start(DRIVING_PIN_FORWARD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /*
        @brief Drive the rear-wheel motors backwards
        at specified PWM duty cycle
    */
    void startMotorsBackward(short dutyCycle){
        pwm_start(DRIVING_PIN_BACKWARD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /*
        @brief stop the forward motor pwm signal
    */
    void stopMotorsForward(){
        pwm_stop(DRIVING_PIN_FORWARD);
    }

    /*
        @brief stop the backward motor pwm signal
    */
    void stopMotorsBackward(){
        pwm_stop(DRIVING_PIN_BACKWARD);
    }
}