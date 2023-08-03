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
constexpr int       JS_THRESHOLD        = 0.015F * 1023;                                                  // Threshold value of joystick, used to avoid drift
const int           JS_BITSHIFT_MAG     = 6;                                                       // Controls bitshift

//  // Potentiometer Values
const int           CIRCLE_RAD_SCALER   = 1;                                                       // Scaler of how big the circle can be
const int           CIRCLE_VEL_SCALER   = 10;
const int           CIRCLE_STEP_SCALER  = 5;

//  // LCD values
const int           LCD_PXL_HEIGHT      = 160;                                                     // Height of the display in pixels
const int           LCD_PXL_WIDTH       = 128;                                                    // Width of the display in pixels

//  // Button
const unsigned long DEBOUNCE_MILLIS     = 200;                                                      // Used to avoid bouncing

//  // Screen
const unsigned long FRAMES_PER_SECOND   = 60;

//  // Circle Physics
constexpr float     GRAVITY             = 9.81F;                                    // g, had to make it constexpr due to C++ shenanigans
const int           STEP_CAP            = 8;
constexpr float     FRICTION_COEFF      = 0.9F;
constexpr float     DENSITY             = 0.0001F;

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

//  // COORDS
#define X                   0
#define Y                   1

//  // Derived Settings
constexpr int           HALF_JOYSTICK_RANGE = (1023 >> 1);
constexpr int           HALF_RANGE          = (1023 >> JS_BITSHIFT_MAG) >> 1;

constexpr unsigned long TIME_DELTA_MILLIS   = 1000 / FRAMES_PER_SECOND;
constexpr float         ACUTAL_TIME_DELTA   = (
    static_cast<float>(TIME_DELTA_MILLIS) / 1000
);

// Debug
const bool DEBUG_MODE       = false;
const bool DEBUG_SCREEN     = false;
const bool DEBUG_TIME       = false;
const bool DEBUG_MOVE       = true;

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
        int         locs[2]     = {LCD_PXL_WIDTH / 2, LCD_PXL_HEIGHT / 2};
        int         radius      = 10;                                                   // Default radius
        uint16_t    colour      = ST7735_CYAN;                                          // Colour of circle
        bool        bFilled     = false;                                                // If the circle is filled

        float       mass        = 10;
        
        float       inertia[2] = {0, 0};
        float       friction    = FRICTION_COEFF * (mass * GRAVITY);

        // Previous Values
        int         prevLocs[2] = {0, 0};
        float       prevVels[2] = {0, 0};

        
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
            
            // Affects Physics
            updatePhysics();
        }

        void changeFilled(bool shouldFill)
        {
            // If False
            if (!shouldFill) return;
            
            // Save Value
            prevBFilled = bFilled;
            bFilled     = !bFilled;
            
            // Affects Physics
            updatePhysics();
        }

        void moveCircle(int movementX, int movementY)
        {
            /*
                This section works using forces (F = MA)
                - M=1
                - Therefore F=A
                - F = sum{Friction, "JS_push"}
            
            */

            // SAVE VALUES
            int steps[2]                = {0, 0};
            int forces[2]               = {0, 0};
            
            float netForces[2]          = {0, 0};
            float vels[2]               = {0, 0};
            float adjustVels[2]         = {0, 0};
            float accelerations[2]      = {0, 0};

            
            // FORCES LOGIC
            //  // Get Velocities
            int moves[2] = {movementX, movementY};

            for (int dir = 0; dir < 2; dir++)
            {
                // SCALER
                forces[dir] = moves[dir] * CIRCLE_VEL_SCALER;
                
                // FORCES (Force = ma = sum{forces}), Axis: Right, Down
                //  // Hold Sign for Friction
                int sign        = (forces[dir] >= 0) ? (1) : (-1);                
                netForces[dir]  = sign * (abs(forces[dir]) - abs(friction));                                       //  // Sign ensures friction is always against force


                // VELOCITY
                //  // Acceleration (a = f/m)
                accelerations[dir]  = netForces[dir] / mass;

                //  // v = u + (a * t)
                adjustVels[dir]     = accelerations[dir] * ACUTAL_TIME_DELTA;
                
                //  // Checks
                bool lessThanFriction   = (abs(netForces[dir]) <= friction);
                bool differentDir       = (
                    (abs(prevVels[dir]) - abs(adjustVels[dir])) <= 0
                );

                if (lessThanFriction && differentDir)
                {
                    vels[dir] = 0;
                }  
                else if (lessThanFriction)
                {
                    int adSign = (prevVels[dir] >= 0.0f) ? (1) : (-1);
                    adjustVels[dir] *= adSign;
                }
                
                //  // Get Velocity
                vels[dir]           = prevVels[dir] + adjustVels[dir];
                
                if (lessThanFriction && (prevVels[dir] == 0)) vels[dir] = 0;

                //  // Save Last Values
                prevVels[dir]  = vels[dir];

                // STEPS
                //  // s = (v * t) -a * (0.5 * t^2)

                steps[dir] = vels[dir] * ACUTAL_TIME_DELTA * CIRCLE_STEP_SCALER;

                //  // Caps
                if (steps[dir] >  STEP_CAP) steps[dir] = HALF_RANGE;
                if (steps[dir] < -STEP_CAP) steps[dir] = -HALF_RANGE;

                //  // Apply Steps
                if (dir == Y) steps[dir] = -steps[dir];                             //  // This fixes the screen
                
                prevLocs[dir]    = locs[dir];
                locs[dir]       += steps[dir];
            }

            //  // Check Limits
            if (locs[0] > LCD_PXL_WIDTH)    locs[0] -= LCD_PXL_WIDTH;                      //  // Resets width if too right
            if (locs[0] < 0)                locs[0] += LCD_PXL_WIDTH;                      //  // Resets width if too left
            
            if (locs[1] > LCD_PXL_HEIGHT)   locs[1] -= LCD_PXL_HEIGHT;                     //  // Resets height if over screen
            if (locs[1] < 0)                locs[1] += LCD_PXL_HEIGHT;                     //  // Resets height if under screen

            // DEBUG
            if (DEBUG_MODE && DEBUG_MOVE)
            {
                if (false)
                {
                Serial.print("Loc: [");
                Serial.print(prevLocs[0]);
                Serial.print(", ");
                Serial.print(prevLocs[1]);
                Serial.print("]");
                }
                if (false)
                {
                Serial.print("\tstep: [");
                Serial.print(steps[0]);
                Serial.print(", ");
                Serial.print(steps[1]);
                Serial.print("]");
                }
                if (false)
                {
                Serial.print("\tmove: [");
                Serial.print(moves[0]);
                Serial.print(", ");
                Serial.print(moves[1]);
                Serial.print("]");
                }
                if (true)
                {
                Serial.print("velocity: {");
                    Serial.print("current: [");
                    Serial.print(vels[0]);
                    Serial.print(", ");
                    Serial.print(vels[1]);
                    Serial.print("]");

                    Serial.print("\tprev: [");
                    Serial.print(prevVels[0]);
                    Serial.print(", ");
                    Serial.print(prevVels[1]);
                    Serial.print("]");
                    
                    Serial.print("\tadjustment: [");
                    Serial.print(adjustVels[0]);
                    Serial.print(", ");
                    Serial.print(adjustVels[1]);
                    Serial.print("]");
                Serial.print("}");
                }
                if (true)
                {
                Serial.print("\tmass&friction: [");
                Serial.print(mass);
                Serial.print(", ");
                Serial.print(friction);
                Serial.print("]");
                }
                if (true)
                {
                Serial.print("\t\t\tforces: {");
                    Serial.print("net: [");
                    Serial.print(netForces[0]);
                    Serial.print(", ");
                    Serial.print(netForces[1]);
                    Serial.print("]");

                    Serial.print("\tforce_joystick: [");
                    Serial.print(forces[0]);
                    Serial.print(", ");
                    Serial.print(forces[1]);
                    Serial.print("]");
                Serial.print("}");
                }
                if (false)
                {
                Serial.print("\t\t\tothers: {");
                    Serial.print("accelerations: [");
                    Serial.print(accelerations[0]);
                    Serial.print(", ");
                    Serial.print(accelerations[1]);
                    Serial.print("]");

                    Serial.print("\tprev_vels: [");
                    Serial.print(prevVels[0]);
                    Serial.print(", ");
                    Serial.print(prevVels[1]);
                    Serial.print("]");
                Serial.print("}");
                }
            }
        }

        void updateFixAndRadius()
        {
            prevBFilled = bFilled;
            prevRadius  = radius;
        }
    private:
        void updatePhysics()
        {
            // Mass
            mass        = (
                (bFilled) ?  (PI*(radius*radius)) : (2*PI*radius)
            ) * DENSITY;

            // Zero Check
            if (mass < 1) mass = 1.0f;

            // Friction
            friction    = FRICTION_COEFF * (mass * GRAVITY);
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
                abs(analogRead(JS_PIN_X) - HALF_JOYSTICK_RANGE) < JS_THRESHOLD
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
                abs(analogRead(JS_PIN_Y) - HALF_JOYSTICK_RANGE) < JS_THRESHOLD
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
            if (abs(value - HALF_JOYSTICK_RANGE) < JS_THRESHOLD) return 0;
            
            // Convert Value
            int convertedValue = (value >> JS_BITSHIFT_MAG) - HALF_RANGE;                        // unsigned int 1024 -> 16, then made signed for -8 to 8

            
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
        uint16_t        colour      = ST7735_BLACK;
        unsigned long   timeDelta   = 0;
        unsigned long   lastTime    = 0;

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

        bool updateTimeDelta()
        {
            int timeCurrent         = millis();
            timeDelta               = timeCurrent - lastTime;
            
            bool pastTimeThreshold  = (timeDelta > TIME_DELTA_MILLIS);

            if (pastTimeThreshold)
            {
                lastTime            = timeCurrent;
                return true;
            }
            
            else return false;
        }

    private:
        // Only used ones
        void printMultipleCircle(Circle& circle, Adafruit_ST7735& chip)
        {
            int printLocX   = circle.locs[0];
            int printLocY   = circle.locs[1];
            int printRadius = circle.radius;

            bool hitLeft    = (printLocX - printRadius) <= 0;
            bool hitRight   = (printLocX + printRadius) >= LCD_PXL_WIDTH;
            bool hitBottom  = (printLocY + printRadius) >= LCD_PXL_HEIGHT;
            bool hitTop     = (printLocY - printRadius) <= 0;

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
                Serial.print(printLocX);
                Serial.print(" Y=");
                Serial.print(printLocY);
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
            int cleanLocX   = circle.prevLocs[0];
            int cleanLocY   = circle.prevLocs[1];
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

            circle.updateFixAndRadius();
        }

        // Individual Circles
        void printCircle(Circle& circle, Adafruit_ST7735& chip,
            int moveX=0, int moveY=0)
        {
            if (circle.bFilled) chip.fillCircle(
                (circle.locs[0] + moveX), 
                (circle.locs[1] + moveY), 
                circle.radius, 
                circle.colour
            ); 
            else chip.drawCircle(
                (circle.locs[0] + moveX), 
                (circle.locs[1] + moveY), 
                circle.radius, 
                circle.colour
            );
        }
        
        void cleanCircle(Circle& circle, Adafruit_ST7735& chip,
            int locX, int locY, int radius)
        {
            // Avoids Cleaning Circles just to Place Them Again
            bool sameLoc    = (
                    (circle.prevLocs[0] == circle.locs[0]) 
                &&  (circle.prevLocs[1] == circle.locs[1])
            );
            bool sameRadius = (circle.prevRadius == circle.radius);
            bool sameFill   = (circle.prevBFilled == circle.bFilled);
            
            if (sameLoc && sameRadius && sameFill) return;

            // Clean Circle
            if (!sameFill || circle.bFilled) chip.fillCircle(
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
    if (DEBUG_MODE && DEBUG_TIME)
    {
        Serial.print("time_d: ");
        Serial.print(screen.timeDelta);
        Serial.print("\ttime_c: ");
        Serial.println(screen.timeDelta);
    }

    bool allowLoop = screen.updateTimeDelta();
    if (allowLoop)
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
        
        // DEBUG
        if (DEBUG_MODE) Serial.println();
    }
}

// NOTES
/*
    - The supply voltage is 5v but the LED voltage is 3.3v
    - Flickering caused by limitations, either make second buffer for screen,
      or faster processor
*/
