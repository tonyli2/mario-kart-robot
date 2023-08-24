namespace Hivemind
{
    enum robotState {START, IR, COAST, TAPE, JUMP};

    void wakeUpHivemind();

    void setupHivemind();

    void setServo(uint8_t angle);

    bool canExitIRFollowing();

} // namespace Hivemind


