#include <fft.h>

namespace FastFourierTransform {

    //FFT Handler
    arduinoFFT FFT = arduinoFFT();

    // Stores sampled readings of signal
    double_t real[SAMPLE]; // Stores real part of transformed signal
    double_t imag[SAMPLE]; // Won't need imag part, but we need a placeholder for function call

    // Timing variables
    unsigned int sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
    unsigned long microseconds;

    double_t fft_start(int analogReadPin)
    {
        /*SAMPLING*/
        microseconds = micros();
        for (uint16_t i = 0; i < SAMPLE; i++)
        {
            real[i] = analogRead(analogReadPin); // Read incoming signal at that moment
            imag[i] = 0;                         // Placeholder

            while (micros() - microseconds < sampling_period_us)
            {
                // Wait so we can satisfy sampling period
            }
            microseconds += sampling_period_us;
        }

        // Now use abstract libraries to transform our signal
        FFT.Windowing(real, SAMPLE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);       /* Abstract library, required but not sure what it does */
        FFT.Compute(real, imag, SAMPLE, FFT_FORWARD);                        /* Transforms signal, stores in "real" variable */
        FFT.ComplexToMagnitude(real, imag, SAMPLE);                          /* At each point i compute the complex-real
                                                                                plane magnitude to see strength of signals at each frequency */
        double_t strongest = FFT.MajorPeak(real, SAMPLE, samplingFrequency); // Strongest frequency

        return strongest;
    }
}