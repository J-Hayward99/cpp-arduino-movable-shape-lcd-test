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
#define JS_THRESHOLD        2                                                       // Threshold value of joystick, used to avoid drift
#define JS_BITSHIFT_MAG     6                                                       // Controls bitshift

//  // Potentiometer Values
#define CIRCLE_RAD_SCALER   2                                                       // Scaler of how big the circle can be
#define CIRCLE_STEP_SCALER  1
//  // LCD values
#define LCD_PXL_HEIGHT      160                                                     // Height of the display in pixels
#define LCD_PXL_WIDTH       128                                                     // Width of the display in pixels

//  // Button
#define DEBOUNCE_MILLIS     50                                                      // Used to avoid bouncing

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
const int halfRange         = (1024 >> JS_BITSHIFT_MAG) / 2;

// Debug
const bool DEBUG_MODE       = false;

// INITIALISING
//  // Circle
class Circle
{
    public:
        int locX            = LCD_PXL_WIDTH     / 2;                                // Initial X coordinate, middle of screen
        int locY            = LCD_PXL_HEIGHT    / 2;                                // Initial Y coordinate, middle of screen
        
        int prevLocX        = 0;
        int prevLocY        = 0;
        
        int radius          = 10;                                                   // Default radius
        uint16_t colour     = ST7735_CYAN;                                          // Colour of circle
        
        bool bFilled        = false;                                                // If the circle is filled

        // Change Values
        void changeColour(bool shouldChange) 
        {                                                                           // Changes the colour through a iterative conditional list
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
            radius = value * CIRCLE_RAD_SCALER;
        }

        void changeFilled(bool shouldFill)
        {
            if (!shouldFill) return;

            (bFilled) ? (bFilled = true) : (bFilled = false);
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
            
            if (lastPressDuration > DEBOUNCE_MILLIS)
            {
                (readValue) ? (state = true) : (state = false);
            }
            
            lastBounceTime = millis();

            if (DEBUG_MODE) reportValuesToSerial();

            return readValue;
        }

        // For Serial Debug
        void reportValuesToSerial()
        {
            Serial.print("Button: ");
            Serial.print(state);
            Serial.print("\t");

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

            if (DEBUG_MODE) reportValuesToSerial();
        };

        // For Serial Debug
        void reportValuesToSerial()
        {
            Serial.print("X-axis: ");
            Serial.print(valueX);

            Serial.print("\tY-axis: ");
            Serial.print(valueY);

            Serial.print("\tButton Pressed: ");
            Serial.print(switchPressed);
            Serial.print("\t");
        };

    
    private:
        int acquireAndConvertValues(uint8_t pin)
        {
            // Get Value
            int value = analogRead(pin);
            int convertedValue = (value >> JS_BITSHIFT_MAG) - halfRange;           // unsigned int 1024 -> 16, then made signed for -8 to 8

            // Check if Value is in Deadzones
            if (abs(convertedValue) < JS_THRESHOLD) convertedValue = 0;
            
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
            
            if (DEBUG_MODE) reportValuesToSerial();
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
        bool        showDebug   = false;

        void printScreen(Circle& circle, Joystick& joystick, 
            Potentiometer& potentiometer, Adafruit_ST7735& chip)
        {
            cleanMultipleCircle(circle, chip, joystick);
            printMultipleCircle(circle, chip);

            if (showDebug)
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
            bool hitTop     = ((circle.locY + circle.radius) >= LCD_PXL_HEIGHT);
            bool hitBottom  = ((circle.locY - circle.radius) <= 0);
            
            // Main Circle
            printCircle(circle, chip);

            // OPPOSITE SIDES
            //  // Left Condition
            if (hitLeft) printCircle(
                circle,
                chip,
                circle.locX + LCD_PXL_WIDTH, 
                circle.locY
            );

            //  // Right Condition
            if (hitRight) printCircle(
                circle,
                chip,
                circle.locX - LCD_PXL_WIDTH,
                circle.locY
            );

            //  // Top Condition
            if (hitTop) printCircle(
                circle,
                chip,
                circle.locX, 
                circle.locY - LCD_PXL_HEIGHT
            );

            //  // Bottom Condition
            if (hitBottom) printCircle(
                circle,
                chip,
                circle.locX, 
                circle.locY + LCD_PXL_HEIGHT
            );

            // OPPOSITE CORNERS
            //  // Top Left Condition
            if (hitTop && hitLeft) printCircle(
                circle,
                chip,
                circle.locX + LCD_PXL_WIDTH, 
                circle.locY + LCD_PXL_HEIGHT
            );
            
            //  // Top Right Condition
            if (hitTop && hitRight) printCircle(
                circle,
                chip,
                circle.locX - LCD_PXL_WIDTH, 
                circle.locY - LCD_PXL_HEIGHT
            );

            //  // Bottom Left Condition
            if (hitBottom && hitLeft) printCircle(
                circle,
                chip,
                circle.locX - LCD_PXL_WIDTH, 
                circle.locY + LCD_PXL_HEIGHT
            );

            // Bottom Right Condition
            if (hitBottom && hitRight) printCircle(
                circle,
                chip,
                circle.locX - LCD_PXL_WIDTH, 
                circle.locY + LCD_PXL_HEIGHT
            );
        }

        void cleanMultipleCircle(const Circle& circle, Adafruit_ST7735& chip,
            Joystick& joystick)
        {
            bool hitLeft    = ((circle.locX - circle.radius) <= 0);
            bool hitRight   = ((circle.locX + circle.radius) >= LCD_PXL_WIDTH);
            bool hitTop     = ((circle.locY + circle.radius) >= LCD_PXL_HEIGHT);
            bool hitBottom  = ((circle.locY - circle.radius) <= 0);

            int cleanLocX   = circle.prevLocX;
            int cleanLocY   = circle.prevLocY;
            
            // Main Circle
            cleanCircle(circle, chip, cleanLocX, cleanLocY);

            // OPPOSITE SIDES
            //  // Left Condition
            if (hitLeft) cleanCircle(
                circle,
                chip,
                cleanLocX + LCD_PXL_WIDTH, 
                cleanLocY
            );

            //  // Right Condition
            if (hitRight) cleanCircle(
                circle,
                chip,
                cleanLocX - LCD_PXL_WIDTH,
                cleanLocY
            );

            //  // Top Condition
            if (hitTop) cleanCircle(
                circle,
                chip,
                cleanLocX, 
                cleanLocY - LCD_PXL_HEIGHT
            );

            //  // Bottom Condition
            if (hitBottom) cleanCircle(
                circle,
                chip,
                cleanLocX, 
                cleanLocY + LCD_PXL_HEIGHT
            );

            // OPPOSITE CORNERS
            //  // Top Left Condition
            if (hitTop && hitLeft) cleanCircle(
                circle,
                chip,
                cleanLocX + LCD_PXL_WIDTH, 
                cleanLocY + LCD_PXL_HEIGHT
            );
            
            //  // Top Right Condition
            if (hitTop && hitRight) cleanCircle(
                circle,
                chip,
                cleanLocX - LCD_PXL_WIDTH, 
                cleanLocY - LCD_PXL_HEIGHT
            );

            //  // Bottom Left Condition
            if (hitBottom && hitLeft) cleanCircle(
                circle,
                chip,
                cleanLocX - LCD_PXL_WIDTH, 
                cleanLocY + LCD_PXL_HEIGHT
            );

            // Bottom Right Condition
            if (hitBottom && hitRight) cleanCircle(
                circle,
                chip,
                cleanLocX - LCD_PXL_WIDTH, 
                cleanLocY + LCD_PXL_HEIGHT
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
        
        void cleanCircle(const Circle& circle, Adafruit_ST7735& chip,
            int moveX=0, int moveY=0)
        {
            if (circle.bFilled) chip.fillCircle(
                moveX, 
                moveY, 
                circle.radius, 
                SCREEN_COLOUR
            ); 
            else chip.drawCircle(
                moveX, 
                moveY, 
                circle.radius, 
                SCREEN_COLOUR
            );
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
        screen.showDebug = true;
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
