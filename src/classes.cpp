// INCLUDES
#include "classes.h"

// DEFINITIONS
// Derived Settings
constexpr int           HALF_JOYSTICK_RANGE = (1023 >> 1);
constexpr int           HALF_RANGE          = (1023 >> JS_BITSHIFT_MAG) >> 1;

constexpr unsigned long TIME_DELTA_MILLIS   = 1000 / FRAMES_PER_SECOND;
constexpr float         ACUTAL_TIME_DELTA   = (
    static_cast<float>(TIME_DELTA_MILLIS) / 1000
);


// CLASSES
// Circle
void Circle::changeColour(bool shouldChange) 
{
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
void Circle::changeRadius(int value)
{
    prevRadius  = radius;
    radius      = value * CIRCLE_RAD_SCALER;
    
    // Affects Physics
    updatePhysics();
}
void Circle::changeFilled(bool shouldFill)
{
    // If False
    if (!shouldFill) return;
    
    // Save Value
    prevBFilled = bFilled;
    bFilled     = !bFilled;
    
    // Affects Physics
    updatePhysics();
}
void Circle::moveCircle(int movementX, int movementY)
{
    
    // SAVE VALUES
    int steps[2]                = {0, 0};                                           //  // Steps of Axis
    int forces[2]               = {0, 0};                                           //  // "Push" forces of axis from joystick
    
    float netForces[2]          = {0, 0};                                           //  // The net force per iteration
    float vels[2]               = {0, 0};                                           //  // The velocity (used to calculate step)
    float adjustVels[2]         = {0, 0};                                           //  // The difference from u and v (useful to seperate)

    
    // FORCES LOGIC
    // Get Velocities
    int moves[2] = {movementX, movementY};

    for (int dir = 0; dir < 2; dir++)
    {
        // SCALER
        forces[dir] = moves[dir] * CIRCLE_VEL_SCALER;
        

        // FORCES (Force = ma = sum{forces}), Axis: Right, Down
        // Hold Sign for Friction
        int sign        = (forces[dir] >= 0) ? (1) : (-1);                
        netForces[dir]  = sign * (abs(forces[dir]) - abs(friction));                //  // Sign ensures friction is always against force


        // VELOCITY
        // Acceleration (a = f/m), Suvat: v = u + (a * t)
        adjustVels[dir] = netForces[dir] / mass * ACUTAL_TIME_DELTA;
        
        // Breaking Friction
        bool lessThanFriction   = (abs(netForces[dir]) <= friction);                //  // If the net force is less than friction it would stop
        bool differentDir       = (
            (abs(prevVels[dir]) - abs(adjustVels[dir])) <= 0);                      //  // Checks the adjustment velocity changes the sign

        if (lessThanFriction && differentDir) 
            vels[dir] = 0;
        else if (lessThanFriction)
            adjustVels[dir] *= (prevVels[dir] >= 0.0f) ? (1) : (-1);
        
        // Get Velocity
        vels[dir] = prevVels[dir] + adjustVels[dir];
        
        // Stops Jittering
        if (lessThanFriction && (prevVels[dir] == 0)) vels[dir] = 0;

        // Save Last Values
        prevVels[dir]  = vels[dir];


        // STEPS
        steps[dir] = vels[dir] * ACUTAL_TIME_DELTA * CIRCLE_STEP_SCALER;            //  // A step = vt (Times a scaler because of issues)

        //  // Cap Max Stepping
        if (steps[dir] >  STEP_CAP) steps[dir] = HALF_RANGE;
        if (steps[dir] < -STEP_CAP) steps[dir] = -HALF_RANGE;

        //  // Apply Steps
        if (dir == Y) steps[dir]    = -steps[dir];                                  //  // This fixes the inverse applied at the start
        prevLocs[dir]               = locs[dir];
        locs[dir]                   += steps[dir];
    }

    //  // CHECK BORDERS
    if (locs[0] > LCD_PXL_WIDTH)    locs[0] -= LCD_PXL_WIDTH;                       //  // Resets width if too right
    if (locs[0] < 0)                locs[0] += LCD_PXL_WIDTH;                       //  // Resets width if too left
    
    if (locs[1] > LCD_PXL_HEIGHT)   locs[1] -= LCD_PXL_HEIGHT;                      //  // Resets height if over screen
    if (locs[1] < 0)                locs[1] += LCD_PXL_HEIGHT;                      //  // Resets height if under screen
}
void Circle::updateFixAndRadius()
{
    prevBFilled = bFilled;
    prevRadius  = radius;
}
void Circle::updatePhysics()
{
    // Mass (Perimeter vs Area)
    mass = ((bFilled) ? (PI*(radius*radius)) : (2*PI*radius)) * DENSITY;

    // Zero Check
    if (mass < 1) mass = 1.0f;

    // Friction
    friction = FRICTION_COEFF * (mass * GRAVITY);
}

// Button
bool Button::updateAndReturnValue(uint8_t pin)
{
    int             readValue         = digitalRead(pin);
    unsigned long   lastPressDuration = (millis() - lastBounceTime);
    
    // Debounce
    if ((lastPressDuration > DEBOUNCE_MILLIS) && (readValue != state))
    {
        state           = readValue;
        lastBounceTime  = millis();
        return state;
    }

    return false;
}

// Joystick
void Joystick::updateValues()
{
    valueX          = acquireAndConvertValues(JS_PIN_X);
    valueY          = acquireAndConvertValues(JS_PIN_Y);
    switchPressed   = digitalRead(JS_PIN_SW);
}
int Joystick::acquireAndConvertValues(uint8_t pin)
{
    // Get Value
    int value = analogRead(pin);

    // Check if Value is in Deadzones
    if (abs(value - HALF_JOYSTICK_RANGE) < JS_THRESHOLD) return 0;
    
    // Convert Value
    int convertedValue = (value >> JS_BITSHIFT_MAG) - HALF_RANGE;                   //  // unsigned int 1024 -> 16, then made signed for -8 to 8

    // Convert Value
    return convertedValue;
}

// Potentiometer
void Potentiometer::updateValue()
{
    value = analogRead(POTENTIOMETER_PIN) / 100;
}

// Screen
void Screen::printScreen(Circle& circle, Joystick& joystick, 
    Potentiometer& potentiometer, Adafruit_ST7735& chip)
{
    cleanMultipleCircle(circle, chip, joystick);
    printMultipleCircle(circle, chip);
}
bool Screen::updateTimeDelta()
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
void Screen::printMultipleCircle(Circle& circle, Adafruit_ST7735& chip)
{
    int printLocX   = circle.locs[0];
    int printLocY   = circle.locs[1];
    int printRadius = circle.radius;

    bool hitLeft    = (printLocX - printRadius) <= 0;
    bool hitRight   = (printLocX + printRadius) >= LCD_PXL_WIDTH;
    bool hitBottom  = (printLocY + printRadius) >= LCD_PXL_HEIGHT;
    bool hitTop     = (printLocY - printRadius) <= 0;

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
void Screen::cleanMultipleCircle(Circle& circle, Adafruit_ST7735& chip,
    Joystick& joystick)
{
    int cleanLocX   = circle.prevLocs[0];
    int cleanLocY   = circle.prevLocs[1];
    int cleanRadius = circle.prevRadius;
    
    bool hitLeft    = ((cleanLocX - cleanRadius)  <= 0);
    bool hitRight   = ((cleanLocX + cleanRadius)  >= LCD_PXL_WIDTH);
    bool hitBottom  = ((cleanLocY + cleanRadius)  >= LCD_PXL_HEIGHT);
    bool hitTop     = ((cleanLocY - cleanRadius)  <= 0);

    
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
void Screen::printCircle(Circle& circle, Adafruit_ST7735& chip,
    int moveX, int moveY)
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
void Screen::cleanCircle(Circle& circle, Adafruit_ST7735& chip,
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
