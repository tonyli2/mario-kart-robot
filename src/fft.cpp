#include <fft.h>

namespace FFT {

    //FFT Handler
    arduinoFFT FFTHandler = arduinoFFT();

    // Stores sampled readings of signal
    double_t real[SAMPLE]; // Stores real part of transformed signal
    double_t imag[SAMPLE]; // Won't need imag part, but we need a placeholder for function call
    
    // Define an array so we can return the magnitude of the 
    // strongest freq as well as its magnitude
    // Index 0: strongest frequency
    // Index 1: magnitude of strongest frequency
    double_t freqAndMagnitude[] = {0, 0};

    const double_t DEVIATION = 20;
    const double_t DESIRED_SIGNAL = 1000.0;

    // Timing variables
    unsigned int sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
    unsigned long microseconds;

    // Private function
    double_t maxMagFinder(double_t *mags);

    double_t * runFFT(int readPin) {

        /*SAMPLING*/
        microseconds = micros();
        for (uint16_t i = 0; i < SAMPLE; i++)
        {
            real[i] = analogRead(readPin); // Read incoming signal at that moment
            imag[i] = 0;                         // Placeholder

            while (micros() - microseconds < sampling_period_us)
            {
                // Wait so we can satisfy sampling period
            }
            microseconds += sampling_period_us;
        }

        // Now use abstract libraries to transform our signal
        FFTHandler.Windowing(real, SAMPLE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);       /* Abstract library, required but not sure what it does */
        FFTHandler.Compute(real, imag, SAMPLE, FFT_FORWARD);                        /* Transforms signal, stores in "real" variable */
        FFTHandler.ComplexToMagnitude(real, imag, SAMPLE);                          /* At each point i compute the complex-real
                                                                                plane magnitude to see strength of signals at each frequency */
        freqAndMagnitude[0] = FFTHandler.MajorPeak(real, SAMPLE, samplingFrequency); // Strongest frequency
        freqAndMagnitude[1] = maxMagFinder(real);
        return freqAndMagnitude;
    }


    bool hasFoundBeacon(int leftPin, int rightPin) {

        double_t *leftReading = runFFT(leftPin);
        double_t *rightReading = runFFT(rightPin);

        bool leftHasDetected = leftReading[0] > DESIRED_SIGNAL + DEVIATION 
                            || leftReading[0] < DESIRED_SIGNAL - DEVIATION;

        bool rightHasDetected = rightReading[0] > DESIRED_SIGNAL + DEVIATION 
                            || rightReading[0] < DESIRED_SIGNAL - DEVIATION;

        return leftHasDetected && rightHasDetected;
    }

    double_t maxMagFinder(double_t *mags) {

        double_t max = 0;
        for (uint32_t i = 0; i < SAMPLE; i++) {
            if (mags[i] > max) {
                max = real[i];
            }
        }
        return max;
    }
}