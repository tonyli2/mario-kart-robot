#include <fft.h>

namespace FFT {

    const double_t MIN_MAG_SUM = 8000;
    const double_t DEVIATION = 100;
    const double_t DESIRED_FREQ = 1000;

    // Timing variables
    unsigned int sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
    unsigned long microseconds;

    // Private function
    float_t maxMagFinder(double_t *mags);

    /**
     * @brief 
     * 
     * @param readPin 
     * @param handler 
     * @return float_t* 
     */
    float_t runFFT(int readPin, arduinoFFT handler) {

        // Stores sampled readings of signal
        double_t real[SAMPLE]; // Stores real part of transformed signal
        double_t imag[SAMPLE]; // Stores imaginary part of transformed signal
        double_t freq[SAMPLE]; // Stores frequencies associated with magnitudes
        double_t sumOfMagnitudes = 0;
    
        handler = arduinoFFT();
        
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
        handler.Windowing(real, SAMPLE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);       /* Abstract library, required but not sure what it does */
        handler.Compute(real, imag, SAMPLE, FFT_FORWARD);                        /* Transforms signal, stores in "real" variable */
        handler.ComplexToMagnitude(real, imag, SAMPLE);                          /* At each point i compute the complex-real
                                                                                plane magnitude to see strength of signals at each frequency */
        
        // Maps magnitude at each element to a frequency. Scalped from arduinoFFT.h
        for(uint16_t i = 1; i < SAMPLE - 1; i++) {

            double delta = 0.5 * ((real[i - 1] - real[i + 1]) /
            (real[i - 1] - (2.0 * real[i]) + real[i + 1]));
        
            double interpolatedX = ((i + delta) * samplingFrequency) / (SAMPLE - 1);

            freq[i] = interpolatedX;

            if(abs(DESIRED_FREQ - interpolatedX) < DEVIATION) {
                sumOfMagnitudes += real[i];
            }
        }

        if(sumOfMagnitudes > MIN_MAG_SUM){            
            return sumOfMagnitudes;
        }

        return 0; // If we do not see 1kHz
    }


    bool hasFoundBeacon(int leftPin, int rightPin, float_t *leftInput, float_t *rightInput,
                        arduinoFFT leftHandler, arduinoFFT rightHandler) {

        float_t leftMag = runFFT(leftPin, leftHandler);
        float_t rightMag = runFFT(rightPin, rightHandler);

        bool leftHasDetected = (leftMag != 0);
        bool rightHasDetected = (rightMag != 0);

        if(!leftHasDetected && !rightHasDetected) {
            return false;
        }

        *leftInput = leftMag;
        *rightInput = rightMag;

        return true;
    }

    float_t maxMagFinder(double_t *mags) {

        double_t max = 0;
        for (uint32_t i = 0; i < SAMPLE; i++) {
            if (mags[i] > max) {
                max = mags[i];
            }
        }
        return max;
    }
}