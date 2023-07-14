namespace DigitalPID {

    void setupServo();

    String applyPID();

    static short calcError(double left, double right);

    static String processOutput(double output);
}