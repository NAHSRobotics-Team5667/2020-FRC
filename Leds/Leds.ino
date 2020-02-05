#include <FastLED.h>

#define NUM_LEDS 120
#define DATA_PIN 11
#define FLASH_TIME 500
#define COLOR_ORDER GRB
#define MAX_BRIGHTNESS    255

CRGB leds[NUM_LEDS];

String state;
CRGB allianceColor;

void setup() {
  FastLED.setBrightness(MAX_BRIGHTNESS);
  FastLED.addLeds<WS2812B, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  Serial.begin(9600);
}

void loop() {
  checkStateAndSetRGBVals();
}
void checkStateAndSetRGBVals(){
  state = Serial.readString();
  if(Serial.readString() == -1){
    allianceColor.r = 255;
    allianceColor.g = 0;
    allianceColor.b = 0;
  }
  else if(Serial.readString() == -2){
    allianceColor.r = 0;
    allianceColor.g = 0;
    allianceColor.b = 255;
  }
  if(state == "0"){ //IDLE
    oneColor(allianceColor.r, allianceColor.g, allianceColor.b); 
  }
  else if(state == "1"){ //AUTO
    flashColor(allianceColor.r, allianceColor.g, allianceColor.b, 0, 0, 0);
  }
  else if(state == "2"){ //DRIVE
    oneColor(255, 0, 255);
  }
  else if(state == "3"){ //VISION
  }
  else if(state == "4"){ //SHOOTING
    flashAndAlternate(255, 0, 255, 255, 255, 255);
  }
  else if(state == "5"){ //CLIMBING
    flashColor(allianceColor.r, allianceColor.g, allianceColor.b, 0, 0, 0);
  }
  else if(state == "6"){ //ROTATION
  }
  else if(state == "7"){ //POSITION
  }
}
void oneColor(int r, int g, int b){
  Serial.println("yayeet");
  for(int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB(r, g, b);
    FastLED.show();
  }
}

void flashColor(int r, int g, int b, int flashR, int flashG, int flashB){
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
  delay(FLASH_TIME);
  for(int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB(flashR, flashG, flashB);
  }
  FastLED.show();
  delay(FLASH_TIME);
}
void flashAndAlternate(int r, int g, int b, int r2, int g2, int b2){
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i % 2 == 0) {
      leds[i] = CRGB(r, g, b);
    }
    else{
      leds[i] = CRGB(r2, g2, b2);
    }
    FastLED.show();
  }
  delay(FLASH_TIME);
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i % 2 == 0) {
      leds[i] = CRGB(r2, g2, b2);
    }
    else{
      leds[i] = CRGB(r, g, b);
    }
    FastLED.show();
  }
  delay(FLASH_TIME);
}
