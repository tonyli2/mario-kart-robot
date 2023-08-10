namespace Hivemind
{
    enum robotState {START, IR, COAST, TAPE, JUMP};

    void wakeUpHivemind();

    void setupHivemind();

    void testServo(uint8_t angle);

    void testMotors();

    bool canExitIRFollowing();

} // namespace Hivemind


