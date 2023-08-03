// LEGAL
// Copyright (c) 2023 James Hayward


// INCLUDES
#include "classes.h"


// NOMENCLATURE
// JS   - Joy Stick


// INITIALISATIONS
// Classes
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
    // Buttons
    pinMode(BUTTON_PIN_COLOUR,  INPUT);
    pinMode(BUTTON_PIN_FILL,    INPUT);

    // Joystick
    pinMode(JS_PIN_X,           INPUT);
    pinMode(JS_PIN_Y,           INPUT);
    pinMode(JS_PIN_SW,          INPUT);

    // Potentiometer
    pinMode(POTENTIOMETER_PIN,  INPUT);

    //SCREEN
    // Communication
    chip.initR(INITR_BLACKTAB);                                                     //  // Sets the background colour
    chip.setSPISpeed(40e6);                                                         //  // Sets the SPI speed (40 million)
    
    //  // Screen Colours
    chip.fillScreen(SCREEN_COLOUR);                                                 //  // Fills the screen black
    chip.setTextColor(ST7735_WHITE, SCREEN_COLOUR);                                 //  // Makes the text white with black background
}

void loop() 
{
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
    }
}

// NOTES
/*
    - The supply voltage is 5v but the LED voltage is 3.3v
    - Flickering caused by limitations, either make second buffer for screen,
      or faster processor
*/
