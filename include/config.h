/*
    This header file contains all the constants
    corresponding to the pins used
*/

// Adafruit OLED Constants
#define SCREEN_WIDTH                    128 // OLED display width, in pixels
#define SCREEN_HEIGHT                   64 // OLED display height, in pixels
#define OLED_RESET                      -1    // This display does not have a reset pin accessible

// Tape Reflectance Sensor L R
#define LEFT_TAPE_PIN                   PA0
#define RIGHT_TAPE_PIN                  PA1



// IR sensors
#define IR_DETECTOR_LEFT                PA3

// Steering servo PWM pin
#define STEERING_SERVO                  PA_10

// Motor driving PWM pins
#define DRIVING_PIN_FORWARD             PA_8
#define DRIVING_PIN_BACKWARD            PA_9
