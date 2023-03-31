// LEGAL
// Copyright (c) 2023 James Hayward


// INCLUDES
#include <Adafruit_GFX.h>                                                           // Used to perform drawings
#include <Adafruit_ST7735.h>                                                        // Hardware specific header, ST7735 is the chipset used
#include <stdbool.h>                                                                // Adds boolean terms


// DEFINITIONS
//  // Drawing Values
#define SCREEN_COLOUR       ST7735_BLACK                                            // Background colour of screen

//  // Joystick Values
#define JS_THRESHOLD        1                                                       // Threshold value of joystick, used to avoid drift

//  // Potentiometer Values
#define CIRCLE_RAD_SCALER   2                                                       // Scaler of how big the circle can be

//  // LCD values
#define LCD_PXL_HEIGHT      160                                                     // Height of the display in pixels
#define LCD_PXL_WIDTH       128                                                     // Width of the display in pixels

//  // Pins
#define BUTTON_PIN_COLOUR   2                                                       // Button to change colour of circle
#define BUTTON_PIN_FILL     3                                                       // Button to fill the circle

#define JOYSTICK_PIN_X      A1                                                      // Analog pin, x direction
#define JOYSTICK_PIN_Y      A0                                                      // Analog pin, y direction
#define JOYSTICK_PIN_SW     7                                                       // Digital pin, unused switch button

#define LCD_PIN_SCK         13                                                      // 13 is the SPI pin on the Arduino Nano
#define LCD_PIN_SDA         11                                                      // 11 is the MOSI pin on the Arduino Nano
#define LCD_PIN_A0          10                                                      // Was selected to group pins numerically
#define LCD_PIN_RST         9                                                       // Was selected to group pins numerically
#define LCD_PIN_CS          8                                                       // Was selected to group pins numerically

#define POTENTIOMETER_PIN   A7                                                      // Analog pin


// INITIALISING
//  // Instances
Adafruit_ST7735 tft = Adafruit_ST7735(LCD_PIN_CS, LCD_PIN_A0, LCD_PIN_RST);         // Initiates instance

//  // Circle
int circleScreenLocX    = LCD_PXL_WIDTH/2;                                          // Initial X coordinate, middle of screen
int circleScreenLocY    = LCD_PXL_HEIGHT/2;                                         // Initial Y coordinate, middle of screen
int circleRadius        = 10;                                                       // Default radius
int circleColour        = ST77XX_CYAN;                                              // Colour of circle
int circleMovementStep  = 2;                                                        // Circle movement speed
bool circleFill         = false;                                                    // If the circle is filled

//  // Buttons
int buttonColour        = false;                                                    // Is the button pressed
int buttonColourState   = false;                                                    // Used to hold the state

int buttonFill          = false;                                                    // Is the button pressed
int buttonFillState     = false;                                                    // Used to hold the state


// PROTOTYPES
void produceCircle(int locX, int locY);                                             // Modularises the production of the circle
void cleanCircle(int locX, int locY);                                               // Modularises the cleaning of the circle
void produceMultipleCircles();                                                      // Handles the circle and fake-circles
void cleanMultipleCircles();                                                        // Handles the circle and fake-circles
void changeColour();                                                                // Handles the colour changing of the circle


// SETUP
void setup() {
    // PINS
    //  // Buttons
    pinMode(BUTTON_PIN_COLOUR,  INPUT);
    pinMode(BUTTON_PIN_FILL,    INPUT);

    //  // Joystick
    pinMode(JOYSTICK_PIN_X,     INPUT);
    pinMode(JOYSTICK_PIN_Y,     INPUT);
    pinMode(JOYSTICK_PIN_SW,    INPUT);

    //  // Potentiometer
    pinMode(POTENTIOMETER_PIN,  INPUT);
    
    // SCREEN
    //  // Communication Links
    tft.initR(INITR_BLACKTAB);                                                      //  // Sets the background colour
    tft.setSPISpeed(40000000);                                                      //  // Sets the SPI speed (40 million)
    
    //  // Screen Colours
    tft.fillScreen(SCREEN_COLOUR);                                                  //  // Fills the screen black
    tft.setTextColor(ST7735_WHITE, SCREEN_COLOUR);                                  //  // Makes the text white with black background
}


// MAIN LOOP
void loop() {
    // CLEARING SCREEN
    cleanMultipleCircles();
    tft.fillRect(20,0, 20,30, SCREEN_COLOUR);                                       //  // This clears the text left by the debug

    // INITIALISATION
    //  // Potentiometer Value Acquiring
    int potentiometerValue  = analogRead(POTENTIOMETER_PIN) / 100;                  //  // Gets value of potentiometer, makes 0 to 10
    circleRadius            = potentiometerValue * CIRCLE_RAD_SCALER;               //  // Changes circle size by scaler

    //  // Joystick Value Acquiring
    int joystickValueX      = (analogRead(JOYSTICK_PIN_X) / 100) - 6;               //  // Range between 0-1024, turns into -6 to 6
    int joystickValueY      = (analogRead(JOYSTICK_PIN_Y) / 100) - 6;               //  // Range between 0-1024, turns into -6 to 6

    //  // Handling Joystick Deadzones
    if ((joystickValueX <= JS_THRESHOLD) && (joystickValueX >= -JS_THRESHOLD)) {
        joystickValueX = 0;
    }                                                                               //  // This is to stop drifting
    
    if ((joystickValueY <= JS_THRESHOLD) && (joystickValueY >= -JS_THRESHOLD)) {
        joystickValueY = 0;
    }                                                                               //  // This is to stop drifting

    //  // Step Calculations
    int nextStepX           =   joystickValueX * circleMovementStep;                //  // Next step for X
    int nextStepY           = - joystickValueY * circleMovementStep;                //  // Next step for Y

    // PLACEMENT LOGIC
    //  // Initiation
    circleScreenLocX += nextStepX;                                                  //  // Applies step X
    circleScreenLocY += nextStepY;                                                  //  // Applies step Y

    //  // Border Handling                                                          
    if (circleScreenLocY > LCD_PXL_HEIGHT) {
        circleScreenLocY -= LCD_PXL_HEIGHT;
    }                                                                               //  // Resets height if over screen
    if (circleScreenLocY < 0) {
        circleScreenLocY += LCD_PXL_HEIGHT;
    }                                                                               //  // Resets height if under screen

    if (circleScreenLocX > LCD_PXL_WIDTH) {
        circleScreenLocX -= LCD_PXL_WIDTH;
    }                                                                               //  // Resets width if too right
    if (circleScreenLocX < 0) {
        circleScreenLocX += LCD_PXL_WIDTH;
    }                                                                               //  // Resets width if too left
    
    // PRODUCING CIRCLE
    produceMultipleCircles();                                                       //  // Produces the circle
    
    // DEBUG TEXT
    tft.setCursor(0,0);
    tft.print("jx: ");
    tft.println(joystickValueX);

    tft.print("jy: ");
    tft.println(joystickValueY);

    tft.print("po: ");
    tft.println(potentiometerValue);

    // BUTTON INPUTS
    //  // Read Values
    buttonColour    = digitalRead(BUTTON_PIN_COLOUR);                               //  // Read the input of the button used for colour
    buttonFill      = digitalRead(BUTTON_PIN_FILL);                                 //  // Read the input of the button used for fill

    //  // Colour Changing Button
    if (buttonColour == true){                                                      //  // If button pressed
        // FUNCTION
        changeColour();                                                             //  // Change colour

        // DEBOUNCER
        buttonColourState = true;                                                   //  // Hold state until the button is released
    }
    else {
        buttonColourState = false;                                                  //  // Allow button to activate function again
    }

    //  // Fill Circle Button
    if (buttonFill == true){                                                        //  // If button pressed
        // FUNCTIONS
        if (circleFill == false) {circleFill = true;}                               //  // Alternate from filled to not or vice versa
        else {circleFill = false;}

        // DEBOUNCER
        buttonFillState = true;                                                     //  // Hold state until the button is released
    }
    else {
        buttonFillState = false;                                                    //  // Allow button to activate function again
    }

}


// FUNCTIONS
void produceCircle(int locX, int locY) {                                            // Produces circle, makes easier to alter
    if (circleFill == true) {
        tft.fillCircle(locX, locY, circleRadius, circleColour);
    } 
    else {
        tft.drawCircle(locX, locY, circleRadius, circleColour);
    }
}

void cleanCircle(int locX, int locY) {                                              // Produces circle, makes easier to alter
    tft.fillCircle(locX, locY, circleRadius+1, SCREEN_COLOUR);
}

void produceMultipleCircles() {                                                     // Handles the multiple circles
    // CONDITIONS
    //  // Left Condition
    if ((circleScreenLocX - circleRadius) <= 0) {
        produceCircle(circleScreenLocX + LCD_PXL_WIDTH, circleScreenLocY);
    }

    //  // Right Condition
    if ((circleScreenLocX + circleRadius) >= LCD_PXL_WIDTH) {
        produceCircle(circleScreenLocX - LCD_PXL_WIDTH, circleScreenLocY);
    }

    //  // Top Condition
    if ((circleScreenLocY + circleRadius) >= LCD_PXL_HEIGHT) {
        produceCircle(circleScreenLocX, circleScreenLocY - LCD_PXL_HEIGHT);
    }

    //  // Bottom Condition
    if ((circleScreenLocY - circleRadius) <= 0) {
        produceCircle(circleScreenLocX, circleScreenLocY + LCD_PXL_HEIGHT);
    }

    //  // Top Left Condition
    if (    (circleScreenLocX - circleRadius)   <= 0
        &&  (circleScreenLocY + circleRadius)   >= LCD_PXL_HEIGHT) {
            produceCircle(circleScreenLocX + LCD_PXL_WIDTH, 
            circleScreenLocY + LCD_PXL_HEIGHT);
    }

    //  // Top Right Condition
    if (    (circleScreenLocX + circleRadius)   >= LCD_PXL_WIDTH
        &&  (circleScreenLocY + circleRadius)   >= LCD_PXL_HEIGHT) {
            produceCircle(circleScreenLocX - LCD_PXL_WIDTH, 
            circleScreenLocY - LCD_PXL_HEIGHT);
    }

    //  // Bottom Left Condition
    if (    (circleScreenLocX - circleRadius)   <= 0
        &&  (circleScreenLocY - circleRadius)   <= 0) {
            produceCircle(circleScreenLocX - LCD_PXL_WIDTH, 
            circleScreenLocY + LCD_PXL_HEIGHT);
    }

    // Bottom Right Condition
    if (    (circleScreenLocX + circleRadius)   >= LCD_PXL_WIDTH
        &&  (circleScreenLocY - circleRadius)   <= 0) {
            produceCircle(circleScreenLocX - LCD_PXL_WIDTH, 
            circleScreenLocY + LCD_PXL_HEIGHT);
    }

    //  // Actual Circle
    produceCircle(circleScreenLocX, circleScreenLocY);
}

void cleanMultipleCircles() {                                                       // Handles the multiple circles
    // CONDITIONS
    //  // Left Condition
    if ((circleScreenLocX - circleRadius) <= 0) {
        cleanCircle(circleScreenLocX + LCD_PXL_WIDTH, circleScreenLocY);
    }

    //  // Right Condition
    if ((circleScreenLocX + circleRadius) >= LCD_PXL_WIDTH) {
        cleanCircle(circleScreenLocX - LCD_PXL_WIDTH, circleScreenLocY);
    }

    //  // Top Condition
    if ((circleScreenLocY + circleRadius) >= LCD_PXL_HEIGHT) {
        cleanCircle(circleScreenLocX, circleScreenLocY - LCD_PXL_HEIGHT);
    }

    //  // Bottom Condition
    if ((circleScreenLocY - circleRadius) <= 0) {
        cleanCircle(circleScreenLocX, circleScreenLocY + LCD_PXL_HEIGHT);
    }

    //  // Top Left Condition
    if (    (circleScreenLocX - circleRadius)   <= 0
        &&  (circleScreenLocY + circleRadius)   >= LCD_PXL_HEIGHT) {
            cleanCircle(circleScreenLocX + LCD_PXL_WIDTH, 
            circleScreenLocY + LCD_PXL_HEIGHT);
    }

    //  // Top Right Condition
    if (    (circleScreenLocX + circleRadius)   >= LCD_PXL_WIDTH
        &&  (circleScreenLocY + circleRadius)   >= LCD_PXL_HEIGHT) {
            cleanCircle(circleScreenLocX - LCD_PXL_WIDTH, 
            circleScreenLocY - LCD_PXL_HEIGHT);
    }

    //  // Bottom Left Condition
    if (    (circleScreenLocX - circleRadius)   <= 0
        &&  (circleScreenLocY - circleRadius)   <= 0) {
            cleanCircle(circleScreenLocX - LCD_PXL_WIDTH, 
            circleScreenLocY + LCD_PXL_HEIGHT);
    }

    // Bottom Right Condition
    if (    (circleScreenLocX + circleRadius)   >= LCD_PXL_WIDTH
        &&  (circleScreenLocY - circleRadius)   <= 0) {
            cleanCircle(circleScreenLocX - LCD_PXL_WIDTH, 
            circleScreenLocY + LCD_PXL_HEIGHT);
    }

    //  // Actual Circle
    cleanCircle(circleScreenLocX, circleScreenLocY);
}

void changeColour() {                                                               // Changes the colour through a iterative conditional list
    if (buttonColourState == false) {
        // CONDITIONS
        //  // Cyan to Magenta
        if (circleColour == ST7735_CYAN) {
            circleColour = ST7735_MAGENTA;
        }

        //  // Magenta to Yellow
        else if (circleColour == ST7735_MAGENTA) {
            circleColour = ST7735_YELLOW;
        }

        //  // Yellow to Green
        else if (circleColour == ST7735_YELLOW) {
            circleColour = ST7735_GREEN;
        }

        //  // Green to White
        else if (circleColour == ST7735_GREEN) {
            circleColour = ST7735_WHITE;
        }

        //  // White to Cyan
        else {
            circleColour = ST7735_CYAN;
        }
    }
}


// NOTES
/*
    - The supply voltage is 5v but the LED voltage is 3.3v
    - Flickering caused by limitations, either make second buffer for screen,
      or faster processor
*/