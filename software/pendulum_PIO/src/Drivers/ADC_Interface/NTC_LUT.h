#include "stdint.h"

/** Credit to Chat GPT :)
 * 
 * NTC info can be found in the ODESC4p2 pinout excel sheet */

#define TABLE_SIZE 23

// Example ADC counts and corresponding temperatures in Celsius
const uint32_t adc_table[TABLE_SIZE] = {
    54,
    98,
    168,
    273,
    423,
    623,
    874,
    1168,
    1492,
    1826,
    2151,
    2454,
    2723,
    2956,
    3153,
    3316,
    3450,
    3559,
    3648,
    3721,
    3780,
    3829,
    3868,
};

const float temperature_table[TABLE_SIZE] = {
    -40.0f,
    -30.0f,
    -20.0f,
    -10.0f,
    0.0f,
    10.0f,
    20.0f,
    30.0f,
    40.0f,
    50.0f,
    60.0f,
    70.0f,
    80.0f,
    90.0f,
    100.0f,
    110.0f,
    120.0f,
    130.0f,
    140.0f,
    150.0f,
    160.0f,
    170.0f,
    180.0f,
};

// Fast lookup function using linear interpolation
static inline float getTemperature(uint32_t adc_count) {
    if (adc_count <= adc_table[0]) {
        return temperature_table[0];  // Return temperature for the lowest ADC count
    }

    for (int i = 1; i < TABLE_SIZE; i++) {
        if (adc_count < adc_table[i]) {
            // Linear interpolation between adc_table[i-1] and adc_table[i]
            float slope = (temperature_table[i] - temperature_table[i-1]) / float(adc_table[i] - adc_table[i-1]);
            return temperature_table[i-1] + slope * (adc_count - adc_table[i-1]);
        }
    }

    return temperature_table[TABLE_SIZE - 1];  // Return temperature for the highest ADC count
}

