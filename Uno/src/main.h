#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "music.h"

#define IN_PIN 0
#define PIXEL_COUNT 24
#define PIXEL_PIN 11
#define PIXEL_READ_PIN 13
#define SPEAKER_PIN1 8
#define SPEAKER_PIN2 12

#define UPPER_NOTE 1000
#define LOWER_NOTE 100
#define NOTE_DURATION 5

int note = 100;
unsigned int nextTime;

enum sirenState{
  UP,
  DOWN
};

sirenState s = UP;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void rainbowCycle(uint8_t wait);
uint32_t Wheel(byte WheelPos);
void off();
void colorWipe(uint32_t c, uint8_t wait);
void playSiren();
