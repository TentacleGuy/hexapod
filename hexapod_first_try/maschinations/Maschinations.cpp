#include "Maschinations.h"

Maschinations::Maschinations(uint8_t numStrips, uint16_t numPixelsPerStrip, uint8_t pins[])
    : _numStrips(numStrips), _isRunning(false), _lastUpdate(0) {
    _strips = new Adafruit_NeoPixel[numStrips];
    for (uint8_t i = 0; i < numStrips; i++) {
        _strips[i] = Adafruit_NeoPixel(numPixelsPerStrip, pins[i], NEO_GRB + NEO_KHZ800);
    }
}

Maschinations::~Maschinations() {
    delete[] _strips;
}

void Maschinations::begin() {
    for (uint8_t i = 0; i < _numStrips; i++) {
        _strips[i].begin();
        _strips[i].show(); // Initialisieren aller Pixel auf 'aus'
    }
}

void Maschinations::update() {
    // Hier könnte die Logik zur Aktualisierung der Animationen stehen
    // Beispiel: Überprüfen, ob eine Animation läuft und entsprechend aktualisieren
    if (_isRunning) {
        // Führen Sie hier die aktuelle Animation aus
    }
}

void Maschinations::bootup(int params[]) {
    _isRunning = true;
    // Implementierung der Bootup-Animation
    // Beispiel: Einfache Animation, die alle LEDs nacheinander einschaltet
    for (uint8_t strip = 0; strip < _numStrips; strip++) {
        for (uint16_t pixel = 0; pixel < _strips[strip].numPixels(); pixel++) {
            _strips[strip].setPixelColor(pixel, _strips[strip].Color(params[0], params[1], params[2]));
            _strips[strip].show();
            delay(params[3]);
        }
    }
    _isRunning = false;
}

void Maschinations::carousel(int params[]) {
    _isRunning = true;
    // Implementierung der Carousel-Animation
    // Beispiel: Einfache Animation, die alle LEDs in einer Schleife durchläuft
    for (uint16_t pixel = 0; pixel < _strips[0].numPixels(); pixel++) {
        for (uint8_t strip = 0; strip < _numStrips; strip++) {
            _strips[strip].setPixelColor(pixel, _strips[strip].Color(params[0], params[1], params[2]));
        }
        for (uint8_t strip = 0; strip < _numStrips; strip++) {
            _strips[strip].show();
        }
        delay(params[3]);
    }
    _isRunning = false;
}

void Maschinations::rainbow() {
    _isRunning = true;
    // Implementierung der Idle-Animation
    // Beispiel: Einfache Animation, die einen Farbwechsel durchführt
    for (uint16_t pixel = 0; pixel < _strips[0].numPixels(); pixel++) {
        uint32_t color = Wheel(pixel & 255, 0); // Verwenden Sie die Wheel-Funktion für Farbwechsel
        for (uint8_t strip = 0; strip < _numStrips; strip++) {
            _strips[strip].setPixelColor(pixel, color);
        }
    }
    for (uint8_t strip = 0; strip < _numStrips; strip++) {
        _strips[strip].show();
    }
    _isRunning = false;
}

void Maschinations::off() {
    for (uint8_t strip = 0; strip < _numStrips; strip++) {
        for (uint16_t pixel = 0; pixel < _strips[strip].numPixels(); pixel++) {
            _strips[strip].setPixelColor(pixel, _strips[strip].Color(0, 0, 0));
        }
        _strips[strip].show();
    }
}

bool Maschinations::isRunning() {
    return _isRunning;
}

uint32_t Maschinations::Wheel(byte WheelPos, int n) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return _strips[n].Color(255 - WheelPos * 3, 0, WheelPos * 3);
    } else if (WheelPos < 170) {
        WheelPos -= 85;
        return _strips[n].Color(0, WheelPos * 3, 255 - WheelPos * 3);
    } else {
        WheelPos -= 170;
        return _strips[n].Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
}
