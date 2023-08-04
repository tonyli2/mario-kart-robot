#include <arduinoFFT.h>

namespace FFT {

    static const uint16_t SAMPLE = 64;
    static const double samplingFrequency = 10000;
    // How many data points you will take on the signal. Experimental number
    // Frequency at which we collect these SAMPLE number of data points.
    //  This is the max frequency that we can detect,
    //(in lab 10kHz is the highest we will see)
    //  The lowest frequency detectable is samplingFrequency/SAMPLE
    //  (in lab we need to be able to detect at least 1kHz)

    // Function definition
    float_t runFFT(int readPin, arduinoFFT handler);
    bool hasFoundBeacon(int leftPin, int rightPin, float_t *leftInput, float_t *rightInput,
                        arduinoFFT leftHandler, arduinoFFT rightHandler);
}