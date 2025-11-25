#include "opt0.h"

#include <Arduino.h>
namespace opt0mode {
void initDataBus() {
    // Init the pinmodes of the data bus for limit switch mode
    pinMode(PC6, OUTPUT_AF_OD);  // Set PC6 as open drain output for limit switch output signal
    // pinMode(PC5, INPUT_PULLDOWN);  // Set PC5 as input with pulldown for halt signal
}

uint32_t averageReadings[5] = {0};
uint8_t readingCount = 0;

uint32_t GetAveragedDistance(uint32_t newReading) {
    // Store the new reading in the array
    averageReadings[readingCount] = newReading;
    readingCount = (readingCount + 1) % 5;  // Increment and wrap around

    // Calculate the average
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 5; i++) {
        sum += averageReadings[i];
    }
    return sum / 5;
}
}  // namespace opt0mode