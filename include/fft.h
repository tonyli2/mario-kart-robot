#include <arduinoFFT.h>

namespace FFT {

    static const uint16_t SAMPLE = 64;
    static const double samplingFrequency = 10000;

    // Function definition
    float_t runFFT(int readPin, arduinoFFT handler);
    bool hasFoundBeacon(int leftPin, int rightPin, float_t *leftInput, float_t *rightInput,
                        arduinoFFT leftHandler, arduinoFFT rightHandler);
}