#ifndef _beatdetector_h // include guard
#define _beatdetector_h

#include <Arduino.h>
#include <Wire.h>
// FHT, see http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // amount of bins to use


#define FreqLog // use log scale for FHT frequencies
#define FreqGainFactorBits 0
#define FreqSerialBinary
#define VolumeGainFactorBits 0

// Macros for faster sampling, see
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


// Set to true if you want to use the FHT 128 channel analyser to visualize
// the detected frequencies. Will disable beat detection.
const bool LOG_FREQUENCY_DATA = false;

// Set to true if the light should be based on detected beats instead
// of detected amplitudes.
const bool PERFORM_BEAT_DETECTION = false;

const int SOUND_REFERENCE_PIN = 8; // D8
const int HAT_LIGHTS_PIN = 6; // D9
const int HAT_LIGHTS_LOW_PIN = 5; // D11
const int HAT_LIGHTS_HIGH_PIN = 4; // D12
const int HAT_LIGHTS_PULSE_PIN = 7; // D13

const int LIGHT_PULSE_DELAY = 10000;
const int LIGHT_PULSE_DURATION = 2000;

const int LIGHT_FADE_OUT_DURATION = 500; // good value range is [100:1000]
const float MINIMUM_LIGHT_INTENSITY = 0.01; // in range [0:1]
const float MAXIMUM_LIGHT_INTENSITY = 0.2; // in range [0:1]

const int MAXIMUM_SIGNAL_VALUE = 1024;

const int OVERALL_FREQUENCY_RANGE_START = 2; // should be 0, but first 2 bands produce too much noise
const int OVERALL_FREQUENCY_RANGE_END = FHT_N / 2;
const int OVERALL_FREQUENCY_RANGE = OVERALL_FREQUENCY_RANGE_END - OVERALL_FREQUENCY_RANGE_START;

const int FIRST_FREQUENCY_RANGE_START = 2;
const int FIRST_FREQUENCY_RANGE_END = 4;
const int FIRST_FREQUENCY_RANGE = FIRST_FREQUENCY_RANGE_END - FIRST_FREQUENCY_RANGE_START;

const int SECOND_FREQUENCY_RANGE_START = 2;
const int SECOND_FREQUENCY_RANGE_END = 6;
const int SECOND_FREQUENCY_RANGE = SECOND_FREQUENCY_RANGE_END - SECOND_FREQUENCY_RANGE_START;

const int MAXIMUM_BEATS_PER_MINUTE = 200;
const int MINIMUM_DELAY_BETWEEN_BEATS = 60000L / MAXIMUM_BEATS_PER_MINUTE;
const int SINGLE_BEAT_DURATION = 100; // good value range is [50:150]

const int FREQUENCY_MAGNITUDE_SAMPLES = 5; // good value range is [5:15]




class Beatdetector {
    public:
    Beatdetector(int analog_input_pin);
    bool init();
    int update();

    private:
    void readAudioSamples();
    void getFrequencyData();
    void processFrequencyData();
    void processOverallFrequencyMagnitude();
    void processFirstFrequencyMagnitude();
    void processSecondFrequencyMagnitude();
    void updateBeatProbability();
    void updateLights();
    int getIntensity();
    void updateLightIntensityBasedOnBeats();
    void updateLightIntensityBasedOnAmplitudes();
    void processHistoryValues(byte history[], int &historyIndex, int &current, int &total, int &average, int &variance);
    void logValue(String name, float value, int length);
    void logValue(String name, float value);
    void logValue(String name, boolean value);
    void logFrequencyData();
    byte getFrequencyMagnitude(byte frequencies[], const int startIndex, const int endIndex);
    float calculateMagnitudeChangeFactor();
    float calculateVarianceFactor();
    float calculateRecencyFactor();
    float calculateSignalChangeFactor();
    String toProgressBar(float value, const int length);


    int frequencyMagnitudeSampleIndex = 0;
    int currentOverallFrequencyMagnitude = 0;
    int totalOverallFrequencyMagnitude = 0;
    int averageOverallFrequencyMagnitude = 0;
    int overallFrequencyMagnitudeVariance = 0;
    byte overallFrequencyMagnitudes[FREQUENCY_MAGNITUDE_SAMPLES];

    int currentFirstFrequencyMagnitude = 0;
    int totalFirstFrequencyMagnitude = 0;
    int averageFirstFrequencyMagnitude = 0;
    int firstFrequencyMagnitudeVariance = 0;
    byte firstFrequencyMagnitudes[FREQUENCY_MAGNITUDE_SAMPLES];

    int currentSecondFrequencyMagnitude = 0;
    int totalSecondFrequencyMagnitude = 0;
    int averageSecondFrequencyMagnitude = 0;
    int secondFrequencyMagnitudeVariance = 0;
    byte secondFrequencyMagnitudes[FREQUENCY_MAGNITUDE_SAMPLES];

    int currentSignal = 0;
    int totalSignal = 0;
    int averageSignal = 0;
    int signalVariance = 0;
    byte signals[FREQUENCY_MAGNITUDE_SAMPLES];

    long lastBeatTimestamp = 0;
    long durationSinceLastBeat = 0;
    float beatProbability = 0;
    float beatProbabilityThreshold = 0.5;

    long lightIntensityBumpTimestamp = 0;
    float lightIntensityBumpValue = 0;
    float lightIntensityValue = 0;

    long lastPulseTimestamp = 0;


    int input_pin_audio= A0;


};

#endif