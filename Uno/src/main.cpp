#include "main.h"

void setup(){
  Serial.begin(9600);
  pinMode(IN_PIN, INPUT);
  pinMode(SPEAKER_PIN1, OUTPUT);
  pinMode(SPEAKER_PIN2, OUTPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  nextTime = millis() + NOTE_DURATION;
}

void loop(){
  Serial.println(digitalRead(IN_PIN));
  if(digitalRead(IN_PIN) == 1){
    rainbowCycle(0);
    playSiren();
  }
  else colorWipe(strip.Color(0, 0, 0), 0);
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

void off(){
  for(uint16_t i=0; i< strip.numPixels(); i++) {
    strip.setPixelColor(i, 255);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void playSiren(){
  switch(s){
    case UP:
      tone(SPEAKER_PIN1, note);
      tone(SPEAKER_PIN2, note);
      if(millis() > nextTime){
        note--;
        nextTime = millis() + NOTE_DURATION;
      }
      if(note == UPPER_NOTE) s = DOWN;
    break;

    case DOWN:
      tone(SPEAKER_PIN1, note);
      tone(SPEAKER_PIN2, note);
      if(millis() > nextTime){
        note--;
        nextTime = millis() + NOTE_DURATION;
      }
      if(note == LOWER_NOTE) s = UP;
    break;
  }
}
