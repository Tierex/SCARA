#ifndef LED_Controller_h
#define LED_Controller_h

    #include <Arduino.h>
    #include <Adafruit_NeoPixel.h>

    
    class LED_Controller {
        public:
            LED_Controller(int Pin);
            void begin();
            void clear();
            void startCycle();
            void setStatus(int status);

        private:
            Adafruit_NeoPixel strip;
            double easeOutCubic(double x);
    };
#endif