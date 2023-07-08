#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <fft.h>

// Constants
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
#define ANALOGPIN PA3    // Analog pin used for detecting frequency

// Set up screen dimensions
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()
{
    pinMode(ANALOGPIN, INPUT);

    display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
    display_handler.display();
    delay(2000);

    // Displays stuff on the screen
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0, 0);
    display_handler.println("Hello...");
    display_handler.display();
}

void loop()
{
    display_handler.clearDisplay();
    display_handler.setCursor(0, 0);
    display_handler.println("Frequency is");
    double_t strongest = fft_start(ANALOGPIN);
    display_handler.println(strongest);\
    display_handler.display();
}

// put function definitions here:
