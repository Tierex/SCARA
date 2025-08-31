#include "LED_Controller.h"

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>


  


LED_Controller::LED_Controller(int pin): strip(10, pin, NEO_GRB + NEO_KHZ800)
{}

double LED_Controller::easeOutCubic(double x) {
  double oneMinusX = 1.0 - x;
  return 1.0 - (oneMinusX * oneMinusX * oneMinusX);
}

void LED_Controller::begin() {
  strip.begin();
  strip.show();
}

void LED_Controller::clear() {
  strip.clear();
  strip.show();
}

void LED_Controller::setStatus(int status) {
  switch(status){
    case 0: //ErrorStop
      strip.fill(strip.Color(200,0,0),0,10);
      break;
    case 1: //Disabled
      strip.fill(strip.Color(26,140,220),0,10);
      break;
    case 2: //STANDBY
      strip.fill(strip.Color(0,200,8),0,10);
      break;
    case 3: //Stopping
      strip.fill(strip.Color(255,100,0),0,10);
      break;
    case 4: //Motion
      strip.fill(strip.Color(0,200,8),0,10);
      break;
    case 5: //Homing
      strip.fill(strip.Color(220,204,0),0,10);
      break;
    default: //-
      strip.fill(strip.Color(26,140,220),0,10);
      break;
  }
  strip.show();
}

void LED_Controller::startCycle() {
  //dim sweep
  // Forward sweep starting from i = 3
  for (int i = 3; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(3, 14, 25));
    strip.show();
    delay(100);
  }

  // Backward sweep from i = 2 to i = 0
  for (int i = 0; i <= 2; i++) {
    strip.setPixelColor(i, strip.Color(3, 14, 25));
    strip.show();
    delay(100);
  }

  //Light up 
  for (double x = 0.0; x <= 1.0; x += 0.05) {
    double easedValue = easeOutCubic(x);
    strip.fill(strip.Color(3+easedValue*24, 14+easedValue*116, 25+easedValue*220));
    strip.show();
    delay(60);
  }
}
