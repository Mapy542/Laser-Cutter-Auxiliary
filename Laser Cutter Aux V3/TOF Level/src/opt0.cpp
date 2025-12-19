#include "opt0.h"

#include <Arduino.h>
namespace opt0mode {
void initDataBus() {
    // Init the pinmodes of the data bus for limit switch mode
    pinMode(PC6, OUTPUT_OD);     // Set PC6 as open drain output for limit switch output signal
                                 // (OUTPUT_AF_OD is not acceptable in pinmode)
    pinMode(PC5, INPUT_PULLUP);  // Set PC5 as input with pulldown for halt signal
}

constexpr uint8_t NUM_READINGS = 2;
uint32_t averageReadings[NUM_READINGS] = {0};
uint8_t readingCount = 0;

uint32_t GetAveragedDistance(uint32_t newReading) {
    // Store the new reading in the array
    averageReadings[readingCount] = newReading;
    readingCount = (readingCount + 1) % NUM_READINGS;  // Increment and wrap around

    // Calculate the average
    uint32_t sum = 0;
    for (uint8_t i = 0; i < NUM_READINGS; i++) {
        sum += averageReadings[i];
    }
    return sum / NUM_READINGS;
}
}  // namespace opt0mode