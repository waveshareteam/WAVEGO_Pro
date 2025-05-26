#ifndef SCREEN_H
#define SCREEN_H

#include <Arduino.h>

// Define the screen width and height
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

class ScreenCtrl {
public:
    ScreenCtrl();
    void init();
    void updateScreen();
    void rotate(int rotateMode);
    void displayText(String text, int16_t x, int16_t y, uint8_t size);
    void update();
    void changeSingleLine(int lineNum, String text, bool updateFlag);
    void updateFrame();
    void clearDisplay();
};

#endif // SCREEN_H