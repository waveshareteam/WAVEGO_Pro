#include "RGBLight.h"
// #include <Adafruit_NeoPixel.h>
#include "Config.h"

RGBLight::RGBLight() : pixels(RGB_NUM, RGB_PIN, NEO_GRB + NEO_KHZ800) {
    // RGB Light initialized
}

void RGBLight::init() {
    // Initialize RGB Light
    pixels.begin();
    pixels.show();
}

void RGBLight::setColor(int id,int r, int g, int b) {
    // Set color of LED
    // id: 0, 1, 2
    // r, g, b: 0-255
    pixels.setPixelColor(id, pixels.Color(g, r, b));
    pixels.show();
}