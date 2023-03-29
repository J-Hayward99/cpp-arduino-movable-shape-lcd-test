// LEGAL
// Copyright (c) 2023 James Hayward


// INCLUDES
#include <Adafruit_GFX.h>                                                           // Used to perform drawings
#include <Adafruit_ST7735.h>                                                        // Hardware specific header, ST7735 is the chipset used


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
#define LCD_PIN_SCK         13                                                      // 13 is the SPI pin on the Arduino Nano
#define LCD_PIN_SDA         11                                                      // 11 is the MOSI pin on the Arduino Nano
#define LCD_PIN_A0          10                                                      // Was selected to group pins numerically
#define LCD_PIN_RST         9                                                       // Was selected to group pins numerically
#define LCD_PIN_CS          8                                                       // Was selected to group pins numerically

#define JOYSTICK_PIN_X      A1                                                      // Analog pin
#define JOYSTICK_PIN_Y      A0                                                      // Analog pin
#define JOYSTICK_PIN_SW     7                                                       // Digital pin

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


// PROTOTYPES
void produceCircle(int locX, int locY, int radius, int colour);
void cleanCircle(int locX, int locY, int radius);

// SETUP
void setup() {
    tft.initR(INITR_BLACKTAB);                                                      //  // Sets the background colour
    tft.setSPISpeed(40000000);                                                      //  // Sets the SPI speed (40 million)
    
    tft.fillScreen(SCREEN_COLOUR);                                                  //  // Fills the screen black
    tft.setTextColor(ST7735_WHITE, SCREEN_COLOUR);                                  //  // Makes the text white with black background
}

// MAIN LOOP
void loop() {
    // CLEARING SCREEN
    //  // Fake Border Circle, Single Corner                                        //  // REVIEW: This can be refactoured
    if ((circleScreenLocY + circleRadius) > LCD_PXL_HEIGHT) {
        cleanCircle(circleScreenLocX, 
            circleScreenLocY - LCD_PXL_HEIGHT, 
            circleRadius
        );
    }                                                                               //  // When it's beyond the height
    if ((circleScreenLocY - circleRadius) < 0) {
        cleanCircle(circleScreenLocX, 
            circleScreenLocY + LCD_PXL_HEIGHT, 
            circleRadius
        );
    }                                                                               //  // When it's below 0
    
    if ((circleScreenLocX + circleRadius) > LCD_PXL_WIDTH) {
        cleanCircle(circleScreenLocX - LCD_PXL_WIDTH, 
            circleScreenLocY, 
            circleRadius
        );
    }                                                                               //  // When it's beyond left
    if ((circleScreenLocX - circleRadius) < 0) {
        cleanCircle(circleScreenLocX + LCD_PXL_WIDTH, 
            circleScreenLocY, 
            circleRadius
        );
    }                                                                               //  // When it's beyond right

    //  // Fake Border Circle, Both Corners                                         //  // REVIEW: This can be refactoured
    if (
            (circleScreenLocX + circleRadius) >= LCD_PXL_WIDTH
        &&  (circleScreenLocY + circleRadius) >= LCD_PXL_HEIGHT
        ) {
        cleanCircle(
            circleScreenLocX - LCD_PXL_WIDTH,
            circleScreenLocY - LCD_PXL_HEIGHT, 
            circleRadius
        );
    }                                                                               //  // When top right, clean bottom left
    if (
            (circleScreenLocX - circleRadius) <= 0
        &&  (circleScreenLocY - circleRadius) <= 0
        ) {
        cleanCircle(
            circleScreenLocX + LCD_PXL_WIDTH,
            circleScreenLocY + LCD_PXL_HEIGHT, 
            circleRadius
        );
    }                                                                               //  // When bottom left, clean top right
    if (
            (circleScreenLocX - circleRadius) <= 0
        &&  (circleScreenLocY + circleRadius) >= LCD_PXL_HEIGHT
        ) {
        cleanCircle(
            circleScreenLocX + LCD_PXL_WIDTH,
            circleScreenLocY - LCD_PXL_HEIGHT, 
            circleRadius
        );
    }                                                                               //  // When top left , clean bottom right
    if (
            (circleScreenLocX + circleRadius) >= LCD_PXL_WIDTH
        &&  (circleScreenLocY - circleRadius) <= 0
        ) {
        cleanCircle(
            circleScreenLocX - LCD_PXL_WIDTH,
            circleScreenLocY + LCD_PXL_HEIGHT, 
            circleRadius
        );
    }                                                                               //  // When bottom right, clean top left
    
    //  // Main Screen Elements
    cleanCircle(circleScreenLocX, circleScreenLocY, circleRadius);                  //  // Clears the current circle
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
    //  // Y Coordinate Handling                                                    //  // REVIEW: This can be refactoured
    if ((circleScreenLocY + circleRadius) >= LCD_PXL_HEIGHT) {
        produceCircle(
            circleScreenLocX,
            circleScreenLocY - LCD_PXL_HEIGHT, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // Makes a fake circle on bottom if exceed top
    if ((circleScreenLocY - circleRadius) <= 0) {
        produceCircle(
            circleScreenLocX,
            circleScreenLocY + LCD_PXL_HEIGHT, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // Makes a fake circle on top if exceed bottom
    
    //  // X Coordinate Handling
    if ((circleScreenLocX + circleRadius) >= LCD_PXL_WIDTH) {
        produceCircle(
            circleScreenLocX - LCD_PXL_WIDTH,
            circleScreenLocY, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // Makes a fake circle on left if exceed right
    if ((circleScreenLocX - circleRadius) <= 0) {
        produceCircle(
            circleScreenLocX + LCD_PXL_WIDTH,
            circleScreenLocY, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // Makes a fake circle on right if exceed left
    
    //  // Both Corners
    if (
            (circleScreenLocX + circleRadius) >= LCD_PXL_WIDTH
        &&  (circleScreenLocY + circleRadius) >= LCD_PXL_HEIGHT
        ) {
        produceCircle(
            circleScreenLocX - LCD_PXL_WIDTH,
            circleScreenLocY - LCD_PXL_HEIGHT, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // When top right, make bottom left
    if (
            (circleScreenLocX - circleRadius) <= 0
        &&  (circleScreenLocY - circleRadius) <= 0
        ) {
        produceCircle(
            circleScreenLocX + LCD_PXL_WIDTH,
            circleScreenLocY + LCD_PXL_HEIGHT, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // When bottom left, make top right
    if (
            (circleScreenLocX - circleRadius) <= 0
        &&  (circleScreenLocY + circleRadius) >= LCD_PXL_HEIGHT
        ) {
        produceCircle(
            circleScreenLocX + LCD_PXL_WIDTH,
            circleScreenLocY - LCD_PXL_HEIGHT, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // When top left, make bottom right
    if (
            (circleScreenLocX + circleRadius) >= LCD_PXL_WIDTH
        &&  (circleScreenLocY - circleRadius) <= 0
        ) {
        produceCircle(
            circleScreenLocX - LCD_PXL_WIDTH,
            circleScreenLocY + LCD_PXL_HEIGHT, 
            circleRadius, 
            circleColour
        );
    }                                                                               //  // When bottom right, make top left

    //  // Actual Circle
    produceCircle(
        circleScreenLocX,
        circleScreenLocY, 
        circleRadius, 
        circleColour
    );                                                                              //  // Produces circle

    // DEBUG TEXT
    tft.setCursor(0,0);
    tft.print("jx: ");
    tft.println(joystickValueX);

    tft.print("jy: ");
    tft.println(joystickValueY);

    tft.print("po: ");
    tft.println(potentiometerValue);


    // DELAY BETWEEN SCREENS
    // delay(10);                                                                      //  // Slows down code

    


}


// FUNCTIONS
void produceCircle(int locX, int locY, int radius, int colour) {
    tft.drawCircle(
        locX, 
        locY, 
        radius, 
        colour
    );
}                                                                                   //  // Produces circle, makes easier to alter

void cleanCircle(int locX, int locY, int radius) {
    tft.fillCircle(
        locX, 
        locY, 
        radius+1, 
        SCREEN_COLOUR
    );
}                                                                                   //  // Cleans circle, makes easier to alter

// NOTES
/*
    - The supply voltage is 5v but the LED voltage is 3.3v
    - Flickering caused by limitations, either make second buffer for screen,
      or faster processor
*/