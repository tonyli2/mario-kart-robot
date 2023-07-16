#include <Servo.h>

namespace DigitalPID {
    struct PID {
        Servo servo;
        const float_t Kp;
        const float_t Ki;
        const float_t Kd;
        const float_t L_THRESHOLD;
        const float_t R_THRESHOLD;
        const uint8_t STRAIGHT_ANGLE;
        const float_t MAX_INTEGRAL;
        float_t leftInput;
        float_t rightInput;
        float_t error;
        float_t prevError;
        float_t derivative;
        float_t integral;
        float_t output;
        uint64_t currTime;
        uint64_t prevTime;
        uint64_t dt;
        const int8_t MAX_ANGLE;
        const int8_t MIN_ANGLE;
    };

    void setupServo();

    String applySteeringPID();

    static void calcSteeringError(float_t *left, float_t *right);

    static String processSteeringOutput(float_t *output);
}