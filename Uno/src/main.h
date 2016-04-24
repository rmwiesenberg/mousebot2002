#include <Arduino.h>
#include "Adafruit_NeoPixel.h"

#define IN_PIN 0
#define PIXEL_COUNT 24
#define PIXEL_PIN 11

void rainbowCycle(uint8_t wait);
uint32_t Wheel(byte WheelPos);
void off();

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
