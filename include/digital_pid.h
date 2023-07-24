#include <Servo.h>

namespace DigitalPID {
    struct PID {
        Servo servo;
        const float_t Kp;
        const float_t Ki;
        const float_t Kd;
        const float_t L_THRESHOLD;      // ADC Threshold voltage (0-1023 analog maps to 0 - 3.3V)
        const float_t R_THRESHOLD;      // ADC Threshold voltage (0-1023 analog maps to 0 - 3.3V)
                                        // On white approx 600
                                        // On black approx 350-450
        const uint8_t STRAIGHT_ANGLE;   // 90 degrees for servo is straight forward
        const float_t MAX_INTEGRAL;     // Integral term max threshold value
        float_t leftInput;              // Current left-wheel reading
        float_t rightInput;             // Current right-wheel reading
        float_t error;                  // PID proportional term
        float_t prevError;              // Previous error for derivative term
        float_t derivative;             // PID derivative term
        float_t integral;               // PID integral term
        float_t output;                 // PID output
        uint64_t currTime;
        uint64_t prevTime;
        uint64_t dt;
        const int8_t MAX_ANGLE;         // Servo max turning angle
        const int8_t MIN_ANGLE;         // Servo min turning angle
    };

    void setupServo(PID *pidType);
    String applyPID(PID *pidType);
    static void calcError(float_t *left, float_t *right, PID *pidType);
    static String processOutput(float_t *output, PID *pidType);
}