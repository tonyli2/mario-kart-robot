#include <fft.h>

namespace FFT {

    /*
        Plan :
        We want to sample the frequencies, then look for a specific band
        centered around 1kHz. Deviation about +-10.

            If the max magnitude within within the frequency band that are above
            a THRESHOLD experimentally determined, hasFoundBeacon should return false
            (if the max in that band isnt over it, none of them will be)

            If we see the maximum frequency within the band is above the threshold, then we have
            found the beacon, and hasFoundBeacon should return true

        Experiment: 

        Determine what the narrowest band around 1kHz. And also what the threshold for the minimum\
        magnitude of the peak within the band should be. To do this we need to
        make measurements from the FFT at different angles from the beacon:
        we record two elements, one for freq other for magnitude corresponding.
        This will allow us to see what the narrowest bandwith would look like. 

        Then we should look at what the magnitude for the peak within the bandwith is when the beacon is off
        This will allow us to determine what noise magnitude looks like, and therefore
        determine what the minimum threshold for the peak should be. Then compare the maximum noise peak magnitude
        to the lowest 1kHz peak magnitude.

    */

    // const double_t MIN_MAG_INDIVIDUAL = 5000;
    const double_t MIN_MAG_SUM = 10000;
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
                // Serial2.print("           Freq: ");
                // Serial2.print(interpolatedX);
                // Serial2.print(":         I M: ");
                // Serial2.println(real[i]);
                sumOfMagnitudes += real[i];
            }
        }

        //  if(readPin == PA4) {
        //         Serial2.print("       Total:");
        //         Serial2.println(sumOfMagnitudes);
        // }

        if(sumOfMagnitudes > MIN_MAG_SUM){
            // if(readPin == PA4) {
            // Serial2.print("       Total:");
            // Serial2.println(sumOfMagnitudes);
            // }
            
            return sumOfMagnitudes;
        }

        return 0; // If we do not see 1kHz
    }


    bool hasFoundBeacon(int leftPin, int rightPin, float_t *leftInput, float_t *rightInput,
                        arduinoFFT leftHandler, arduinoFFT rightHandler) {

        float_t leftMag = runFFT(leftPin, leftHandler);
        float_t rightMag = runFFT(rightPin, rightHandler);

        // Serial2.print(" L: --------> ");
        // Serial2.print(leftMag);
        // Serial2.print(" ||| ");
        // Serial2.print(rightMag);
        // Serial2.println("  <-------- R ");

        bool leftHasDetected = (leftMag != 0);
        bool rightHasDetected = (rightMag != 0);

        if(!leftHasDetected && !rightHasDetected) {
            return false;
        }
        // else if(leftHasDetected && !rightHasDetected){
        //     *leftInput = leftMag;
        //     *rightInput = 0;
        // }
        // else if(!leftHasDetected && rightHasDetected){
        //     *leftInput = 0;
        //     *rightInput = rightMag;
        // }
        // else {
        //     *leftInput = leftMag;
        //     *rightInput = rightMag;
        // }

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