// LEGAL
// Copyright (c) 2023 James Hayward


// INCLUDES
#include <Arduino.h>                                                                // Required for PlatformIO (intellisense seems to hate it's guts though)
#include <Adafruit_ST7735.h>
#include <math.h>


// NOMENCLATURE
// JS   - Joy Stick


// DEFINITIONS
//  // Drawing Values
#define SCREEN_COLOUR       ST7735_BLACK                                            // Background colour of screen

//  // Joystick Values
constexpr int       JS_THRESHOLD        = 0.01f * 1023;                                                  // Threshold value of joystick, used to avoid drift
const int           JS_BITSHIFT_MAG     = 6;                                                       // Controls bitshift

//  // Potentiometer Values
const int           CIRCLE_RAD_SCALER   = 2;                                                       // Scaler of how big the circle can be
const int           CIRCLE_STEP_SCALER  = 1;

//  // LCD values
const int           LCD_PXL_HEIGHT      = 160;                                                     // Height of the display in pixels
const int           LCD_PXL_WIDTH       = 128;                                                    // Width of the display in pixels

//  // Button
const unsigned long DEBOUNCE_MILLIS     = 200;                                                      // Used to avoid bouncing

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

//  // Derived Settings
constexpr int   halfJoystickRange   = (1023 >> 1);
constexpr int   halfRange           = (1023 >> JS_BITSHIFT_MAG) >> 1;

// Debug
const bool DEBUG_MODE       = false;
const bool DEBUG_SCREEN     = false;

const bool DEBUG_PRINT      = false;
const bool DEBUG_JS         = false;
const bool DEBUG_BUTTONS    = false;
const bool DEBUG_KNOB       = false;

// INITIALISING
//  // Circle
class Circle
{
    public:
        // Current Values
        int         locX        = LCD_PXL_WIDTH     / 2;                                // Initial X coordinate, middle of screen
        int         locY        = LCD_PXL_HEIGHT    / 2;                                // Initial Y coordinate, middle of screen
        int         radius      = 10;                                                   // Default radius
        uint16_t    colour      = ST7735_CYAN;                                          // Colour of circle
        bool        bFilled     = false;                                                // If the circle is filled

        // Previous Values
        int         prevLocX    = 0;
        int         prevLocY    = 0;
        int         prevRadius  = 0;
        bool        prevBFilled = false;

        // Change Values
        void changeColour(bool shouldChange) 
        {                                                                           // Changes the colour through a iterative conditional list
            // If False
            if (!shouldChange) return;
            // List starts on Cyan, will cycle through
            // Yes there is probably a better way to do this

            // -- LIST OF COLOURS -- 
            // Cyan
            // Magenta
            // Yellow
            // Green
            // White

            if      (colour == ST7735_CYAN)     colour = ST7735_MAGENTA;
            else if (colour == ST7735_MAGENTA)  colour = ST7735_YELLOW; 
            else if (colour == ST7735_YELLOW)   colour = ST7735_GREEN;
            else if (colour == ST7735_GREEN)    colour = ST7735_WHITE;
            else                                colour = ST7735_CYAN;
        }

        void changeRadius(int value)
        {
            prevRadius  = radius;
            radius      = value * CIRCLE_RAD_SCALER;
        }

        void changeFilled(bool shouldFill)
        {
            // If False
            if (!shouldFill) return;
            
            // Save Value
            prevBFilled = bFilled;
            bFilled     = !bFilled;
        }

        void moveCircle(int movementX, int movementY)
        {
            prevLocX                = locX;
            prevLocY                = locY;

            // Get Movement Steps
            int nextStepX           =   movementX * CIRCLE_STEP_SCALER;             //  // Next step for X
            int nextStepY           = - movementY * CIRCLE_STEP_SCALER;             //  // Next step for Y

            // Apply Steps
            locX += nextStepX;
            locY += nextStepY;

            // Check Limits
            if (locY > LCD_PXL_HEIGHT)  locY -= LCD_PXL_HEIGHT;                     //  // Resets height if over screen
            if (locY < 0)               locY += LCD_PXL_HEIGHT;                     //  // Resets height if under screen

            if (locX > LCD_PXL_WIDTH)   locX -= LCD_PXL_WIDTH;                      //  // Resets width if too right
            if (locX < 0)               locX += LCD_PXL_WIDTH;                      //  // Resets width if too left
        }

        void updateFixAndRadius()
        {
            prevBFilled = bFilled;
            prevRadius  = radius;
        }
};

//  // Controls
class Button
{
    public:
        bool            state             = false;
        unsigned long   lastBounceTime    = 0;

        bool updateAndReturnValue(uint8_t pin)
        {
            int readValue                   = digitalRead(pin);
            unsigned long lastPressDuration = (millis() - lastBounceTime);
            
            // Debug
            if (DEBUG_MODE && DEBUG_BUTTONS) reportValuesToSerial(
                readValue, lastPressDuration);

            // Debounce
            if ((lastPressDuration > DEBOUNCE_MILLIS) && (readValue != state))
            {
                state           = readValue;
                lastBounceTime  = millis();
                return state;
            }

            return false;
        }

        // For Serial Debug
        void reportValuesToSerial(int readValue, 
        unsigned long lastPressDuration)
        {
            Serial.print("Button: [");
            Serial.print(readValue);
            Serial.print(", ");
            Serial.print(state);
            Serial.print(", ");
            Serial.print(lastPressDuration > DEBOUNCE_MILLIS);
            Serial.print(", ");
            Serial.print(lastPressDuration);
            Serial.print(" (");
            Serial.print(DEBOUNCE_MILLIS);
            Serial.print(")]\t\t");

        };
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

        // Used to Get the Latest Value
        void updateValues()
        {
            valueX          = acquireAndConvertValues(JS_PIN_X);
            valueY          = acquireAndConvertValues(JS_PIN_Y);
            switchPressed   = digitalRead(JS_PIN_SW);

            if (DEBUG_MODE && DEBUG_JS) reportValuesToSerial();
        };

        // For Serial Debug
        void reportValuesToSerial()
        {
            Serial.print("X-axis: [");
            Serial.print(analogRead(JS_PIN_X));
            Serial.print(", ");
            Serial.print(analogRead(JS_PIN_X)/1023.0f * 100);
            Serial.print(", ");
            Serial.print(valueX);
            Serial.print("]");
            Serial.print(" (");
            Serial.print(
                abs(analogRead(JS_PIN_X) - halfJoystickRange) < JS_THRESHOLD
            );
            Serial.print(")");

            Serial.print("\tY-axis: [");
            Serial.print(analogRead(JS_PIN_Y));
            Serial.print(", ");
            Serial.print(analogRead(JS_PIN_Y)/1023.0f * 100);
            Serial.print(", ");
            Serial.print(valueY);
            Serial.print("]");
            Serial.print(" (");
            Serial.print(
                abs(analogRead(JS_PIN_Y) - halfJoystickRange) < JS_THRESHOLD
            );
            Serial.print(")");


            Serial.print("\tSW: ");
            Serial.print(switchPressed);
            Serial.print("\t");
        };

    
    private:
        int acquireAndConvertValues(uint8_t pin)
        {
            // Get Value
            int value = analogRead(pin);

            // Check if Value is in Deadzones
            if (abs(value - halfJoystickRange) < JS_THRESHOLD) return 0;
            
            // Convert Value
            int convertedValue = (value >> JS_BITSHIFT_MAG) - halfRange;                        // unsigned int 1024 -> 16, then made signed for -8 to 8

            
            // Convert Value
            return convertedValue;
        };

};

class Potentiometer
{
    public:
        int value = 0;

        void updateValue()
        {
            value = analogRead(POTENTIOMETER_PIN) / 100;
            
            if (DEBUG_MODE && DEBUG_KNOB) reportValuesToSerial();
        }

        // For Serial Debug
        void reportValuesToSerial()
        {
            Serial.print("Knob: ");
            Serial.print(value);
            Serial.print("\t");

        };
};

//  // Screen
class Screen
{
    public:
        uint16_t    colour      = ST7735_BLACK;

        void printScreen(Circle& circle, Joystick& joystick, 
            Potentiometer& potentiometer, Adafruit_ST7735& chip)
        {
            cleanMultipleCircle(circle, chip, joystick);
            printMultipleCircle(circle, chip);

            if (DEBUG_MODE && DEBUG_SCREEN)
            {
                // Clear Debug
                chip.fillRect(20,0, 20,30, SCREEN_COLOUR);

                // Print Debug
                chip.setCursor(0,0);
                chip.print("jx: ");
                chip.println(joystick.valueX);

                chip.print("jy: ");
                chip.println(joystick.valueY);

                chip.print("po: ");
                chip.println(potentiometer.value);
            }

        }
    
    private:
        // Only used ones
        void printMultipleCircle(const Circle& circle, Adafruit_ST7735& chip)
        {
            bool hitLeft    = ((circle.locX - circle.radius) <= 0);
            bool hitRight   = ((circle.locX + circle.radius) >= LCD_PXL_WIDTH);
            bool hitBottom  = ((circle.locY + circle.radius) >= LCD_PXL_HEIGHT);
            bool hitTop     = ((circle.locY - circle.radius) <= 0);

            if (DEBUG_MODE && DEBUG_PRINT)
            {
                Serial.print("\tPRINT {");
                Serial.print("L=");
                Serial.print(hitLeft);
                Serial.print(" R=");
                Serial.print(hitRight);
                Serial.print(" T=");
                Serial.print(hitTop);
                Serial.print(" B=");
                Serial.print(hitBottom);
                Serial.print(" X=");
                Serial.print(circle.locX);
                Serial.print(" Y=");
                Serial.print(circle.locY);
                Serial.print("}");
            }
            // Main Circle
            printCircle(circle, chip);

            // OPPOSITE SIDES
            //  // Left Condition
            if (hitLeft) printCircle(
                circle,
                chip,
                LCD_PXL_WIDTH, 
                0
            );

            //  // Right Condition
            if (hitRight) printCircle(
                circle,
                chip,
                -LCD_PXL_WIDTH,
                0
            );

            //  // Top Condition
            if (hitTop) printCircle(
                circle,
                chip,
                0, 
                LCD_PXL_HEIGHT
            );

            //  // Bottom Condition
            if (hitBottom) printCircle(
                circle,
                chip,
                0, 
                -LCD_PXL_HEIGHT
            );

            // OPPOSITE CORNERS
            //  // Top Left Condition
            if (hitTop && hitLeft) printCircle(
                circle,
                chip,
                LCD_PXL_WIDTH, 
                LCD_PXL_HEIGHT
            );
            
            //  // Top Right Condition
            if (hitTop && hitRight) printCircle(
                circle,
                chip,
                -LCD_PXL_WIDTH, 
                LCD_PXL_HEIGHT
            );

            //  // Bottom Left Condition
            if (hitBottom && hitLeft) printCircle(
                circle,
                chip,
                LCD_PXL_WIDTH, 
                -LCD_PXL_HEIGHT
            );

            // Bottom Right Condition
            if (hitBottom && hitRight) printCircle(
                circle,
                chip,
                -LCD_PXL_WIDTH, 
                -LCD_PXL_HEIGHT
            );
        }

        void cleanMultipleCircle(Circle& circle, Adafruit_ST7735& chip,
            Joystick& joystick)
        {
            int cleanLocX   = circle.prevLocX;
            int cleanLocY   = circle.prevLocY;
            int cleanRadius = circle.prevRadius;

            bool hitLeft    = ((cleanLocX - cleanRadius)  <= 0);
            bool hitRight   = ((cleanLocX + cleanRadius)  >= LCD_PXL_WIDTH);
            bool hitBottom  = ((cleanLocY + cleanRadius)  >= LCD_PXL_HEIGHT);
            bool hitTop     = ((cleanLocY - cleanRadius)  <= 0);


            if (DEBUG_MODE && DEBUG_PRINT)
            {
                Serial.print("CLEAN {");
                Serial.print("L=");
                Serial.print(hitLeft);
                Serial.print(" R=");
                Serial.print(hitRight);
                Serial.print(" T=");
                Serial.print(hitTop);
                Serial.print(" B=");
                Serial.print(hitBottom);
                Serial.print(" X=");
                Serial.print(cleanLocX);
                Serial.print(" Y=");
                Serial.print(cleanLocY);
                Serial.print("}");
            }
            
            // Main Circle
            // FIXME On border, clean doesn't work with rad or fill change
            cleanCircle(circle, chip, cleanLocX, cleanLocY, cleanRadius);

            // OPPOSITE SIDES
            //  // Left Condition
            if (hitLeft) cleanCircle(
                circle,
                chip,
                cleanLocX + LCD_PXL_WIDTH, 
                cleanLocY,
                cleanRadius
            );

            //  // Right Condition
            if (hitRight) cleanCircle(
                circle,
                chip,
                cleanLocX - LCD_PXL_WIDTH,
                cleanLocY,
                cleanRadius
            );

            //  // Top Condition
            if (hitTop) cleanCircle(
                circle,
                chip,
                cleanLocX, 
                cleanLocY + LCD_PXL_HEIGHT,
                cleanRadius
            );

            //  // Bottom Condition
            if (hitBottom) cleanCircle(
                circle,
                chip,
                cleanLocX, 
                cleanLocY - LCD_PXL_HEIGHT,
                cleanRadius
            );

            // OPPOSITE CORNERS
            //  // Top Left Condition
            if (hitTop && hitLeft) cleanCircle(
                circle,
                chip,
                cleanLocX + LCD_PXL_WIDTH, 
                cleanLocY + LCD_PXL_HEIGHT,
                cleanRadius
            );
            
            //  // Top Right Condition
            if (hitTop && hitRight) cleanCircle(
                circle,
                chip,
                cleanLocX - LCD_PXL_WIDTH, 
                cleanLocY + LCD_PXL_HEIGHT,
                cleanRadius
            );

            //  // Bottom Left Condition
            if (hitBottom && hitLeft) cleanCircle(
                circle,
                chip,
                cleanLocX + LCD_PXL_WIDTH, 
                cleanLocY - LCD_PXL_HEIGHT,
                cleanRadius
            );

            // Bottom Right Condition
            if (hitBottom && hitRight) cleanCircle(
                circle,
                chip,
                cleanLocX - LCD_PXL_WIDTH, 
                cleanLocY - LCD_PXL_HEIGHT,
                cleanRadius
            );
        }

        // Individual Circles
        void printCircle(const Circle& circle, Adafruit_ST7735& chip,
            int moveX=0, int moveY=0)
        {
            if (circle.bFilled) chip.fillCircle(
                (circle.locX + moveX), 
                (circle.locY + moveY), 
                circle.radius, 
                circle.colour
            ); 
            else chip.drawCircle(
                (circle.locX + moveX), 
                (circle.locY + moveY), 
                circle.radius, 
                circle.colour
            );
        }
        
        void cleanCircle(Circle& circle, Adafruit_ST7735& chip,
            int locX, int locY, int radius)
        {
            // Avoids Cleaning Circles just to Place Them Again
            bool sameLoc    = (
                    (circle.prevLocX == circle.locX) 
                &&  (circle.prevLocY == circle.locY)
            );
            bool sameRadius = (circle.prevRadius == circle.radius);
            bool sameFill   = (circle.prevBFilled == circle.bFilled);
            
            if (sameLoc && sameRadius && sameFill) return;

            // Clean Circle
            if (circle.prevBFilled) chip.fillCircle(
                locX, 
                locY, 
                radius, 
                SCREEN_COLOUR
            ); 
            else chip.drawCircle(
                locX, 
                locY, 
                radius, 
                SCREEN_COLOUR
            );

            // Turn Off Filled and Radius Fix
            circle.updateFixAndRadius();
        }    
};

// INITIALISATIONS
//  // Classes
Circle          circle;
Joystick        joystick;
Button          nextColour;
Button          fillColour;
Potentiometer   potentiometer;
Screen          screen;
Adafruit_ST7735 chip = Adafruit_ST7735(LCD_PIN_CS, LCD_PIN_A0, LCD_PIN_RST);

// SETUP
void setup() 
{
    // PINS
    //  // Buttons
    pinMode(BUTTON_PIN_COLOUR,  INPUT);
    pinMode(BUTTON_PIN_FILL,    INPUT);

    //  // Joystick
    pinMode(JS_PIN_X,           INPUT);
    pinMode(JS_PIN_Y,           INPUT);
    pinMode(JS_PIN_SW,          INPUT);

    //  // Potentiometer
    pinMode(POTENTIOMETER_PIN,  INPUT);

    //SCREEN
    
    //  // Communication
    chip.initR(INITR_BLACKTAB);                                             //  // Sets the background colour
    chip.setSPISpeed(40e6);                                                 //  // Sets the SPI speed (40 million)
    
    //  // Screen Colours
    chip.fillScreen(SCREEN_COLOUR);                                         //  // Fills the screen black
    chip.setTextColor(ST7735_WHITE, SCREEN_COLOUR);                         //  // Makes the text white with black background
    

    //  // Debug Stuff
    if (DEBUG_MODE)
    {
        Serial.begin(9600);
        Serial.println("Hello World!");
    }
}

void loop() 
{
    // POTENTIOMETER INPUTS
    potentiometer.updateValue();
    circle.changeRadius(potentiometer.value);

    // JOYSTICK INPUTS
    joystick.updateValues();
    circle.moveCircle(joystick.valueX, joystick.valueY);

    // BUTTON INPUTS
    circle.changeColour(nextColour.updateAndReturnValue(BUTTON_PIN_COLOUR));
    circle.changeFilled(fillColour.updateAndReturnValue(BUTTON_PIN_FILL));

    // // CLEARING AND PRINTING SCREEN
    screen.printScreen(circle, joystick, potentiometer, chip);
    
    if (DEBUG_MODE) Serial.println();
}

// NOTES
/*
    - The supply voltage is 5v but the LED voltage is 3.3v
    - Flickering caused by limitations, either make second buffer for screen,
      or faster processor
*/
