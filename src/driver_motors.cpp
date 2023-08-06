#include <config.h>

namespace DriverMotors{

    /*
        @brief Drive the right-wheel motors forward
        at specified PWM duty cycle
    */
    void startMotorsForwardRight(short dutyCycle){
        pwm_start(RIGHT_DRIVING_FWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(RIGHT_DRIVING_BKWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

    }

    /*
        @brief Drive the left-wheel motors forward
        at specified PWM duty cycle
    */
    void startMotorsForwardLeft(short dutyCycle){
        pwm_start(LEFT_DRIVING_FWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(LEFT_DRIVING_BKWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /*
        @brief Drive the right-wheel motors backwards
        at specified PWM duty cycle
    */
    void startMotorsBackwardRight(short dutyCycle){
        pwm_start(RIGHT_DRIVING_BKWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(RIGHT_DRIVING_FWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /*
        @brief Drive the left-wheel motors backwards
        at specified PWM duty cycle
    */
    void startMotorsBackwardLeft(short dutyCycle){
        pwm_start(LEFT_DRIVING_BKWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(LEFT_DRIVING_FWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /*
        @brief stop the right forward motor pwm signal
    */
    void stopMotorsBoth(){
        pwm_start(RIGHT_DRIVING_FWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(RIGHT_DRIVING_BKWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(LEFT_DRIVING_FWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(LEFT_DRIVING_BKWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }


    void diffSteeringLeft(){
        DriverMotors::startMotorsForwardRight(55);
        DriverMotors::startMotorsForwardLeft(35);
    }

    void diffSteeringRight(){
        DriverMotors::startMotorsForwardRight(35);
        DriverMotors::startMotorsForwardLeft(55);
    }

}