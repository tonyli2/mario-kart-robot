#include <arduinoFFT.h>
#include <fft.h>

arduinoFFT FFT = arduinoFFT();

static const uint16_t SAMPLE = 64;             // How many data points you will take on the signal. Experimental number
static const double samplingFrequency = 10000; // Frequency at which we collect these SAMPLE number of data points.
                                               //  This is the max frequency that we can detect,
                                               //(in lab 10kHz is the highest we will see)
                                               //  The lowest frequency detectable is samplingFrequency/SAMPLE
                                               //  (in lab we need to be able to detect at least 1kHz)

unsigned int sampling_period_us;
unsigned long microseconds;

double_t real[SAMPLE]; // Stores real part of transformed signal
double_t imag[SAMPLE]; // Won't need imag part, but we need a placeholder for function call

double_t fft_start(int analogReadPin)
{

    sampling_period_us = round(1000000 * (1.0 / samplingFrequency)); // Convert Frequency to Period in microseconds

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
