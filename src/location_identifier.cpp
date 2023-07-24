#include <config.h>

namespace LocationIdentifier {

    float_t leftMarker = 0.0f;
    float_t rightMarker = 0.0f;
    float_t leftTapeSens = 0.0f;
    float_t rightTapeSens = 0.0f;
    const float_t LM_THRESHOLD = 600.0f;
    const float_t RM_THRESHOLD = 600.0f;
    bool isGoingUpRamp = false;

    /**
     * @brief determines whether the robot has detected the tape marker that
     * indicates an impending jump, and whether it should trigger the
     * jump handler interrupt sequence.
     * 
     * @return true the robot is supposed to trigger the jump handler sequence
     * @return false the robot is not supposed to trigger the jump handler sequence
     */
    bool isReadyToJump(){
        
        leftMarker = analogRead(MARKER_SENSE_LEFT);
        rightMarker = analogRead(MARKER_SENSE_RIGHT);

        leftTapeSens = analogRead(LEFT_TAPE_PIN);
        rightTapeSens = analogRead(RIGHT_TAPE_PIN);

        bool readyToJump = 
            leftMarker < LM_THRESHOLD && 
            rightMarker < RM_THRESHOLD &&
            leftTapeSens < LM_THRESHOLD &&
            rightTapeSens < RM_THRESHOLD &&
            isGoingUpRamp;

        bool reachedRampStart = 
            leftMarker < LM_THRESHOLD && 
            rightMarker > RM_THRESHOLD &&
            leftTapeSens <  LM_THRESHOLD &&
            rightTapeSens < RM_THRESHOLD &&
            !isGoingUpRamp;

        if(readyToJump){
            // Robot has now gone up the ramp and is ready to jump 
            // Reset upRamp marker
            isGoingUpRamp = false;
        } 
        else if(reachedRampStart){
            // Robot hits single left-marker while approaching the up-ramp
            isGoingUpRamp = true;
        }

        return readyToJump;
    }
}