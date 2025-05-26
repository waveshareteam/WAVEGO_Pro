#include "ScreenCtrl.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


String line_1 = "";
String line_2 = "";
String line_3 = "";
String line_4 = "";

// Create an instance of the SSD1306 display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

ScreenCtrl::ScreenCtrl() {
    // Constructor
}

void ScreenCtrl::init() {
    if (!display.begin(0x3C, 0x3C)) { // Default I2C address is 0x3C
        // Initialization failed, handle error here (e.g., log or retry)
        return;
    }
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.display();
}

void ScreenCtrl::rotate(int rotateMode) {
    display.setRotation(rotateMode);
}

void ScreenCtrl::updateScreen() {
    display.clearDisplay();
    display.display();
}

// size = 1：ABCDEFGHIJK0123456789 in one line
void ScreenCtrl::displayText(String text, int16_t x, int16_t y, uint8_t size) {
    display.setCursor(x, y);
    display.setTextSize(size); // Default text size
    display.print(text);
    display.display();
}

void ScreenCtrl::changeSingleLine(int lineNum, String text, bool updateFlag) {
    switch (lineNum) {
        case 1:
            line_1 = text;
            break;
        case 2:
            line_2 = text;
            break;
        case 3:
            line_3 = text;
            break;
        case 4:
            line_4 = text;
            break;
    }
    if (updateFlag) {
        updateFrame();
    }
}

void ScreenCtrl::updateFrame() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print(line_1);
    display.setCursor(0, 8);
    display.print(line_2);
    display.setCursor(0, 16);
    display.print(line_3);
    display.setCursor(0, 24);
    display.print(line_4);
    display.display();
}

void ScreenCtrl::clearDisplay() {
    display.clearDisplay();
    display.display();
}