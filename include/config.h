/*
    This header file contains all the constants
    corresponding to the pins used
*/

// Adafruit OLED Constants
#define SCREEN_WIDTH                    128 // OLED display width, in pixels
#define SCREEN_HEIGHT                   64 // OLED display height, in pixels
#define OLED_RESET                      -1    // This display does not have a reset pin accessible

// Tape Reflectance Sensor L R (Analog)
#define LEFT_TAPE_PIN                   PA0
#define RIGHT_TAPE_PIN                  PA1

// Marker Reflectance Sensors L R (Digital)
#define MARKER_SENSE_LEFT               PB0
#define MARKER_SENSE_RIGHT              PB1
#define INTERRUPT_PIN                   PB7

// IR sensors (Analog)
#define IR_DETECTOR_LEFT                PA4
#define IR_DETECTOR_RIGHT               PA5

// Steering servo pin (PWM)
#define STEERING_SERVO                  PA_10

// Motor driving pins (PWM)
#define DRIVING_PIN_FORWARD             PB_8
#define DRIVING_PIN_BACKWARD            PB_9
