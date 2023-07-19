#include <math.h>

namespace LocationIdentifier {

    float_t leftMarker = 0.0f;
    float_t rightMarker = 0.0f;
    float_t leftTapeSens = 0.0f;
    float_t rightTapeSens = 0.0f;
    const float_t LM_THRESHOLD = 600.0f;
    const float_t RM_THRESHOLD = 600.0f;
    bool isGoingUpRamp = false;

    bool isReadyToJump();
}