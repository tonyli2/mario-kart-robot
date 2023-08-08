#ifndef SERVO.H
#define SERVO.H

#include <Servo.h>

#endif // Guard for servo.h

namespace DigitalPID {
    struct PID {
        const float_t Kp;
        const float_t Ki;
        const float_t Kd;
        const float_t L_THRESHOLD;      // ADC Threshold voltage for left Tape (0-1023 analog maps to 0 - 3.3V)
        const float_t R_THRESHOLD;      // ADC Threshold voltage for right Tape (0-1023 analog maps to 0 - 3.3V)
        const float_t LM_THRESHOLD;     // ADC Threshold voltage for left marker
        const float_t RM_THRESHOLD;     // ADC Threshold voltage for right marker
                                        // On white approx 600
                                        // On black approx 350-450
        float_t IR_THRESHOLD;           // Difference between l & R IR signal for PID
        const uint8_t STRAIGHT_ANGLE;   // Angle for servo that is straight forward
        const float_t MAX_INTEGRAL;     // Integral term max threshold value
        float_t leftTapeInput;          // Current left tape sensor reading
        float_t leftMarkerInput;        // Current left marker sensor reading
        float_t rightTapeInput;         // Current right tape sensor reading
        float_t rightMarkerInput;       // Current right marker sensor reading
        float_t error;                  // PID proportional term
        float_t prevError;              // Previous error for derivative term
        float_t derivative;             // PID derivative term
        float_t integral;               // PID integral term
        float_t output;                 // PID output
        uint64_t currTime;
        uint64_t prevTime;
        uint64_t t0;
        uint64_t dt;
        const int8_t MAX_ANGLE;         // Servo max turning angle
        const int8_t MIN_ANGLE;         // Servo min turning angle
        bool isIR;
        arduinoFFT leftFFTHandler;      // Left FFT computation
        arduinoFFT rightFFTHandler;     // Right FFT computation
        uint8_t TURNING_SPEED;          // Duty cycle during turns
        uint8_t STRAIGHT_SPEED;         // Duty cycle during straights
        bool justEscapedIR;
    };

    void setupServo(Servo servo);
    void applyPID(PID *pidType, Servo servo);
    static void calcError(PID *pidType, bool *applyDifferential);
    static void processOutput(PID *pidType, bool applyDifferential, Servo servo);
}