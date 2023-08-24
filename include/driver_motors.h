#define PWM_FREQ 100

namespace DriverMotors{

    void startMotorsForwardRight(short dutyCycle);

    void startMotorsForwardLeft(short dutyCycle);

    void startMotorsBackwardRight(short dutyCycle);

    void startMotorsBackwardLeft(short dutyCycle);

    void stopMotorsBoth();

    void diffSteering(uint32_t leftSpeed, uint32_t rightSpeed, bool leftTurn);
}