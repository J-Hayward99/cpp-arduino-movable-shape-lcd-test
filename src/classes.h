#ifndef CLASSES_H
#define CLASSES_H

// INCLDUES
#include <Arduino.h>                                                                // Required for PlatformIO (intellisense seems to hate it's guts though)
#include <Adafruit_ST7735.h>                                                        // Required for the screen
#include <math.h>                                                                   // Only used for getting absolute values

//  // Drawing Values
#define SCREEN_COLOUR       ST7735_BLACK                                            // Background colour of screen

//  // Pins
#define BUTTON_PIN_COLOUR   2                                                       // Button to change colour of circle
#define BUTTON_PIN_FILL     3                                                       // Button to fill the circle

#define JS_PIN_X            A1                                                      // Analog pin, x direction
#define JS_PIN_Y            A0                                                      // Analog pin, y direction
#define JS_PIN_SW           7                                                       // Digital pin, unused switch button

#define LCD_PIN_SCK         13                                                      // 13 is the SPI pin on the Arduino Nano
#define LCD_PIN_SDA         11                                                      // 11 is the MOSI pin on the Arduino Nano
#define LCD_PIN_A0          10                                                      // Was selected to group pins numerically
#define LCD_PIN_RST         9                                                       // Was selected to group pins numerically
#define LCD_PIN_CS          8                                                       // Was selected to group pins numerically

#define POTENTIOMETER_PIN   A7                                                      // Analog pin

// COORDS
#define X                   0
#define Y                   1

// Joystick Values
constexpr int       JS_THRESHOLD        = 0.015F * 1023;                            // Threshold value of joystick, used to avoid drift
const int           JS_BITSHIFT_MAG     = 6;                                        // Controls bitshift

// Potentiometer Values
const int           CIRCLE_RAD_SCALER   = 1;                                        // Scaler of how big the circle can be
const int           CIRCLE_VEL_SCALER   = 10;                                       // Scaler of how "fast" the circle moves
const int           CIRCLE_STEP_SCALER  = 5;                                        // Scaler of the steps, used to improve acceleration

// LCD values
const int           LCD_PXL_HEIGHT      = 160;                                      // Height of the display in pixels
const int           LCD_PXL_WIDTH       = 128;                                      // Width of the display in pixels

// Button
const unsigned long DEBOUNCE_MILLIS     = 200;                                      // Used to avoid bouncing

// Screen
const unsigned long FRAMES_PER_SECOND   = 60;                                       // Frames 

// Circle Physics
constexpr float     GRAVITY             = 9.81F;                                    // g, had to make it constexpr due to C++ shenanigans
const int           STEP_CAP            = 8;
constexpr float     FRICTION_COEFF      = 0.9F;
constexpr float     DENSITY             = 0.0001F;

// CLASSES
class Circle
{
    public:
        // Current Values
        uint16_t    colour      = ST7735_CYAN;                                      // Colour of the Circle
        bool        bFilled     = false;                                            // If the Circle is filled
        int         radius      = 10;                                               // Radius of the Circle, gets immidiately overwritten

        int         locs[2]     = {LCD_PXL_WIDTH / 2, LCD_PXL_HEIGHT / 2};          // Location of the Circle
        float       mass        = (
            (mass < 1) ? (1.0f) : (2*PI*radius) * DENSITY
        );                                                                          // Mass of the Circle, I could calculate this but cba tbh
        float       friction    = FRICTION_COEFF * (mass * GRAVITY);                // Friction of the Circle, affects acceleration

        // Previous Values
        int         prevLocs[2] = {0, 0};                                           // Previous locs, used for erasing circles of the Screen
        float       prevVels[2] = {0, 0};                                           // Previous velocities, used for moving the Circle

        int         prevRadius  = 0;                                                // Previous radius, used for erasing circles of the Screen
        bool        prevBFilled = false;                                            // Previous filled state, used for erasing circles of the Screen

        void changeColour(bool shouldChange);
        void changeRadius(int value);
        void changeFilled(bool shouldFill);
        void moveCircle(int movementX, int movementY);
        void updateFixAndRadius();
    
    private:
        void updatePhysics();
};
class Button
{
    public:
        bool            state             = false;                                  // Starting state (and future)
        unsigned long   lastBounceTime    = 0;                                      // Used for debouncing

        bool updateAndReturnValue(uint8_t pin);
};
class Joystick
{
    public:
        // Movement Input
        int valueX = 0;
        int valueY = 0;

        // Previous Values
        int prevValueX = 0;
        int prevValueY = 0;

        // Switch (Future Use)
        bool switchPressed = false;

        void updateValues();
        int acquireAndConvertValues(uint8_t pin);
};
class Potentiometer
{
    public:
        int value = 0;                                                              // Value of the potentiometer, immidiately overwritten

        void updateValue();
};
class Screen
{
    public:
        uint16_t        colour      = ST7735_BLACK;                                 // Background colour
        unsigned long   timeDelta   = 0;                                            // Used for stabilising frame rate
        unsigned long   lastTime    = 0;                                            // Used for stabilising frame rate

        void printScreen(Circle& circle, Joystick& joystick, 
            Potentiometer& potentiometer, Adafruit_ST7735& chip);
        bool updateTimeDelta();
    
    private:
        void printMultipleCircle(Circle& circle, Adafruit_ST7735& chip);
        void cleanMultipleCircle(Circle& circle, Adafruit_ST7735& chip,
            Joystick& joystick);
        
        void printCircle(Circle& circle, Adafruit_ST7735& chip,
            int moveX=0, int moveY=0);
        void cleanCircle(Circle& circle, Adafruit_ST7735& chip,
            int locX, int locY, int radius);
};

#endif // CLASSES_H