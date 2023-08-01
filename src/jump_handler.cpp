#include <config.h>

namespace JumpHandler {

    /**
     * @brief sets up IRQ Handler for Marker Reader
     * reflectance sensors
     * 
     */
    void setupJumpHandler() {
        pinMode(INTERRUPT_PIN, OUTPUT);
        attachInterrupt(INTERRUPT_PIN, jumpHandler, RISING);
    }

    /**
     * @brief interrupt handler which processes logic 
     * required before and after bridge jump
     * 
     */
    void jumpHandler(){
        
        // TODO code for managing bridge jump here

        // 1. Go Straight off bridge
        // DriverMotors::startMotorsForward(30);

        // 2. Trigger IMU sequence (reaction wheel)
        
        // 3. When landed, use IMU to turn robot to face beacon

        // 4. Reset Interrupt Pin
        digitalWrite(INTERRUPT_PIN, LOW);
    }
}
