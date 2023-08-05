namespace JumpHandler {

    extern float_t leftMarker;
    extern float_t rightMarker;
    extern float_t leftTapeSens;
    extern float_t rightTapeSens;
    extern const float_t LM_THRESHOLD;
    extern const float_t RM_THRESHOLD;
    extern bool isGoingUpRamp;

    bool isReadyToJump();

    void setupJumpHandler();

    void jumpHandler();

    void afterJump(bool *doneTurn);

    float_t calcIMUSteering(uint32_t *speed);
}
