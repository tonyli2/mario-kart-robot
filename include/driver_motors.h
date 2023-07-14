#define PWM_FREQ 100

namespace DriverMotors{

    void startMotorsForward(short dutyCycle);

    void startMotorsBackward(short dutyCycle);

    void stopMotorsForward();

    void stopMotorsBackward();
}