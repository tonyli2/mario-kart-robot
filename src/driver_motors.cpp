#include <config.h>

namespace DriverMotors{

    /**
        @brief Drive the right-wheel motors forward
        at specified PWM duty cycle
    */
    void startMotorsForwardRight(short dutyCycle){
        pwm_start(RIGHT_DRIVING_FWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(RIGHT_DRIVING_BKWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /**
        @brief Drive the left-wheel motors forward
        at specified PWM duty cycle
    */
    void startMotorsForwardLeft(short dutyCycle){
        pwm_start(LEFT_DRIVING_FWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(LEFT_DRIVING_BKWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /**
        @brief Drive the right-wheel motors backwards
        at specified PWM duty cycle
    */
    void startMotorsBackwardRight(short dutyCycle){
        pwm_start(RIGHT_DRIVING_BKWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(RIGHT_DRIVING_FWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /**
        @brief Drive the left-wheel motors backwards
        at specified PWM duty cycle
    */
    void startMotorsBackwardLeft(short dutyCycle){
        pwm_start(LEFT_DRIVING_BKWD, PWM_FREQ, dutyCycle, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

        pwm_start(LEFT_DRIVING_FWD, PWM_FREQ, 0, 
        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    }

    /**
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

    /**
     * @brief send PWM signal to motors to syncronize them
     * and perform a differential steer where one wheel reverses
     * while the other speeds up. This causes the robot to 
     * conduct a tight turn. 
     * 
     * @param leftSpeed left motor duty cycle
     * @param rightSpeed right motor duty cycle
     * @param leftTurn true if we want to perform a left turn, false otherwise
     */
    void diffSteering(uint32_t leftSpeed, uint32_t rightSpeed, bool leftTurn){

        if(leftTurn) {
            DriverMotors::startMotorsForwardRight(rightSpeed);
            DriverMotors::startMotorsBackwardLeft(leftSpeed);
        }
        else {
            DriverMotors::startMotorsBackwardRight(rightSpeed);
            DriverMotors::startMotorsForwardLeft(leftSpeed);
        }

    }
}