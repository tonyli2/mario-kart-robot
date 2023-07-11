#include <Arduino.h>

#define LEFT_READ_PIN PA0
#define RIGHT_READ_PIN PA1

// Define PID constants
double Kp = 1.0;  // Proportional gain
double Ki = 0.2;  // Integral gain
double Kd = 0.1;  // Derivative gain

// Define PID variables
double setpoint = 0;   // Desired value
double leftInput = 0;  // Current left-wheel reading
double rightInput = 0; // Current right-wheel reading
double output = 0;     // Output value

// Define variables for calculation
double error = 0;            // Error term
double previousError = 0;    // Previous error term
double integral = 0;         // Integral term

// Define other variables
unsigned long previousTime = 0;  // Previous time for derivative calculation
unsigned long sampleTime = 100;  // Sample time in milliseconds

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set the initial values
  leftInput = analogRead(LEFT_READ_PIN);  // Read the current value from left wheel
  rightInput = analogRead(RIGHT_READ_PIN); // Read the current value from right wheel

  // Set the output range if necessary
  // For example, if you're using a PWM pin, you can set the output range using analogWriteRange()
  // analogWriteRange(255);  // Set the range to 0-255
}

void loop() {
  // Calculate the elapsed time since the last iteration
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - previousTime;

  // Check if the sample time has passed
  if (elapsedTime >= sampleTime) {
    // Read the current value
    leftInput = analogRead(A0);

    // Calculate the error term
    // error = setpoint - input;

    // Calculate the integral term
    integral += error * elapsedTime;

    // Calculate the derivative term
    double derivative = (error - previousError) / elapsedTime;

    // Calculate the PID output
    output = Kp * error + Ki * integral + Kd * derivative;

    // Update the previous error and time for the next iteration
    previousError = error;
    previousTime = currentTime;

    // Apply the output (e.g., adjust a motor speed, LED brightness, etc.)
    // For example, if you're using a PWM pin, you can set the output value using analogWrite()
    // analogWrite(ledPin, output);

    // Print the values for debugging
    Serial.print("Input: ");
    // Serial.print(input);
    Serial.print(" Output: ");
    Serial.print(output);
    Serial.println();

    // Delay for the remaining sample time
    delay(sampleTime - elapsedTime);
  }
}