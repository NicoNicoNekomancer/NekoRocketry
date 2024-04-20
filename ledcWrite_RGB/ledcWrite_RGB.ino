#include <Arduino.h>
#include <Adafruit_NeoPixel.h>


#define PIN        23
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB);
#define DELAYVAL 500
// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pixels.begin();
}

// the loop routine runs over and over again forever:
void loop() {


    pixels.setPixelColor(0, pixels.Color(random(200), random(200), random(200)));
    pixels.show();
    delay(DELAYVAL);
  
}