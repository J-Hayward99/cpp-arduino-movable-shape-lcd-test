# 1 "C:\\Users\\James-Hayward\\Documents\\github\\arduino-movable-shape-lcd-test\\main\\main.ino"
// LEGAL
// Copyright (c) 2023 James Hayward


// SETUP
void setup()
{
    ///Adafruit_QDTech tft = Adafruit_QDTech;
    pinMode(13, 0x1);
}

// MAIN LOOP
void loop()
{
    digitalWrite(13, 0x1);
    delay(2000);
    digitalWrite(13, 0x0);
    delay(200);
}
