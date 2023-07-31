# Arduino: Movable Shape LCD Test

# Content

- [1 Purpose](#1)
- [2 Plan](#2)
- [3 Getting Started](#3)
  - [3.1 Prequisites](#31)
  - [3.2 Set-up](#32)
    - [3.2.1 Nano to Breadboard](#321)
    - [3.2.2 Buttons](#322)
    - [3.2.3 Joystick](#323)
    - [3.2.4 LCD Display](#324)
    - [3.2.5 Potentiometer](#325)
- [4 Built With](#4)
- [5 Notes](#5)
- [6 Authors](#6)

# 1 Purpose <a name="1"></a>

This project was made due to me having some sensors spare and I wanted to
exercise my coding skills with an Arduino again.

# 2 Plan <a name="2"></a>

- Have a shape on the LCD
- Able to control the shape with a joystick
- Able to size the shape with a potentiometer
- Able to alter the shape with a button

# 3 Getting Started <a name="3"></a>

## 3.1 Prequisites <a name="31"></a>

- 1 Arduino Nano
- 1 Standard Arduino Joystick Module
- 1 Potentiometer
- 1 AZDelivery 1.8 inch SPI TFT ST7735 LCD  
- 2 Buttons
- 2 10k Resistors
- N Jumper Cables

## 3.2 Set-up <a name="32"></a>

### 3.2.1 Nano to Breadboard <a name="321"></a>

- Nano plugged into board
- GND to GND rail
- 5V to VCC rail

### 3.2.2 Buttons <a name="322"></a>

- HIGH to VCC rail
- LOW to 10k resistor, which is to the GND rail
- "Colour Change" pin to D2
- " Fill" pin to D3

### 3.2.3 Joystick <a name="323"></a>

- VCC to VCC rail
- GND to GND rail
- "X" pin to A1
- "Y" pin to A2
- "Switch" pin to D7

### 3.2.4 LCD Display <a name="324"></a>

- VCC to VCC rail
- GND to GND rail
- "LED" pin to 3V3
- "SCK" pin to D13 (Nano's SCK pin)
- "SDA" pin to D11 (Nano's MOSI pin)
- "A0" pin to D10
- "RESET" pin to D9
- "CS" pin to D8

### 3.2.5 Potentiometer <a name="325"></a>

- VCC to VCC rail
- GND to GND rail
- "Feedback" pin to A7

## 4 Built With <a name="4"></a>

This code was built in VS Code with the Arduino extension

## 5 Notes <a name="5"></a>

- The supply voltage is 5.0V while the LED voltage is 3.3V
- The flickering effect is caused by processor speed limitations,
  since the processor cannot run fast enough to "hide"
  the elements being drawn one at a time, an improvement would be
  to make a second buffer however I do not own the hardware for that improvement.

## 6 Authors <a name="6"></a>

James Hayward - Sole Author
