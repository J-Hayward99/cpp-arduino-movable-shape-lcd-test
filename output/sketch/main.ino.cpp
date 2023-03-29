#include <Arduino.h>
#line 1 "C:\\Users\\James-Hayward\\Documents\\github\\arduino-movable-shape-lcd-test\\main\\main.ino"
// LEGAL
// Copyright (c) 2023 James Hayward


// SETUP
#line 6 "C:\\Users\\James-Hayward\\Documents\\github\\arduino-movable-shape-lcd-test\\main\\main.ino"
void setup();
#line 13 "C:\\Users\\James-Hayward\\Documents\\github\\arduino-movable-shape-lcd-test\\main\\main.ino"
void loop();
#line 6 "C:\\Users\\James-Hayward\\Documents\\github\\arduino-movable-shape-lcd-test\\main\\main.ino"
void setup()
{
    ///Adafruit_QDTech tft = Adafruit_QDTech;
    pinMode(LED_BUILTIN, OUTPUT);
}

// MAIN LOOP
void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
}
