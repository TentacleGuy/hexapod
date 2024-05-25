#ifndef Maschinations_h
#define Maschinations_h

#include <Adafruit_NeoPixel.h>

class Maschinations {
public:
    Maschinations(uint8_t numStrips, uint16_t numPixelsPerStrip, uint8_t pins[]);
    ~Maschinations();
    void begin();
    void update();
    void bootup(int params[]);
    void carousel(int params[]);
    void rainbow();
    void off();
    bool isRunning();

private:
    Adafruit_NeoPixel* _strips;
    uint8_t _numStrips;
    unsigned long _lastUpdate;
    bool _isRunning;
    uint32_t Wheel(byte WheelPos, int n);
};

#endif
