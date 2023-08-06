#include <config.h>


namespace HallEffect
{
    void setupHalleffect() {
        pinMode(HALL_EFFECT_PIN_R, INPUT_PULLUP);
        pinMode(HALL_EFFECT_PIN_L, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PIN_R), handleInterrupt1, FALLING);
        attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PIN_R), handleInterrupt2, RISING);

        attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PIN_L), handleInterrupt1, FALLING);
        attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PIN_L), handleInterrupt2, RISING);
    }

    void handleInterrupt1() {
        DriverMotors::slowDownMotors();
    }

    void handleInterrupt2() {
        DriverMotors::speedUpMotors();
    }

} // namespace HallEffect
