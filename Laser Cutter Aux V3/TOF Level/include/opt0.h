#ifndef OPT0_H
#define OPT0_H

#include <Arduino.h>

namespace opt0mode {
void initDataBus();
uint32_t GetAveragedDistance(uint32_t newReading);
}  // namespace opt0mode

#endif  // OPT0_H
