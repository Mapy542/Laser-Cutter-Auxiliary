#include "readOpt.h"

#include <Arduino.h>
namespace readOpt {
uint8_t ReadOpt(uint32_t opt0pin, uint32_t opt1pin, uint32_t opt2pin) {
    // Read the state of the option pins and return a 3-bit value
    pinMode(opt0pin, INPUT_PULLUP);
    pinMode(opt1pin, INPUT_PULLUP);
    pinMode(opt2pin, INPUT_PULLUP);

    delay(10);  // Small delay to allow pin states to stabilize

    uint8_t opt0 = !digitalRead(opt0pin);
    uint8_t opt1 = !digitalRead(opt1pin);
    uint8_t opt2 = !digitalRead(opt2pin);

    return (opt2 << 2) | (opt1 << 1) | opt0;  // Combine into a single 3-bit value
}
}  // namespace readOpt