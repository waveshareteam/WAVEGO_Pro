// for PlatformIO. This is the main file of the project.
// This file is the entry point of the program.
#include <Arduino.h>

#include "RGBLight.h"
RGBLight led;

void setup() {
  // put your setup code here, to run once:
  led.init();
  led.setColor(0, 9, 0, 0);
  led.setColor(1, 0, 0, 9);
}

void loop() {
  // put your main code here, to run repeatedly:
  // led.setColor(0, 255, 0, 0);
  // delay(1000);
  // led.setColor(0, 0, 255, 0);
  // delay(1000);
}

