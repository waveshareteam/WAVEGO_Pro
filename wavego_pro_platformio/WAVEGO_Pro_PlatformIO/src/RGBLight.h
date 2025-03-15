#ifndef RGB_LIGHT_H
#define RGB_LIGHT_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class RGBLight {
public:
    RGBLight();  // Constructor
    void setColor(int id, int r, int g, int b);  // Set color of LED
    void init();  // Initialize RGB Light

private:
    Adafruit_NeoPixel pixels;
};

#endif // RGB_LIGHT_H