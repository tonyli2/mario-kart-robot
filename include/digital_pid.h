namespace DigitalPID {
    struct pid {
        const float_t Kp;
        const float_t Ki;
        const float_t Kd;
    };

    void setupServo();

    String applyPID();

    static short calcError(double left, double right);

    static String processOutput(double output);
}