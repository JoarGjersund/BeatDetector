#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <Beatdetector.h>

Beatdetector headbanger = Beatdetector(A7);
void setup() {
  wdt_reset();
  wdt_enable(WDTO_8S);

  pinMode(HAT_LIGHTS_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_LOW_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_HIGH_PIN, OUTPUT);
  pinMode(HAT_LIGHTS_PULSE_PIN, OUTPUT);
  pinMode(SOUND_REFERENCE_PIN, OUTPUT);
  

  headbanger.init();
  
  Serial.begin(115200);
  Serial.println("Starting Festival Hat Controller");
}


void loop() {
  wdt_reset();
  headbanger.update();
}

