#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>

#include "opt0.h"
#include "readOpt.h"

#define FOCALOffset 2
// in mm, sensor to lens height difference

#define LED0 PC7
#define LED1 PD0

#define OPT0_PIN PD4
#define OPT1_PIN PD3
#define OPT2_PIN PD2

uint8_t optSetting = 0;
VL53L1X sensor;

void setup() {
    optSetting = readOpt::ReadOpt(OPT0_PIN, OPT1_PIN, OPT2_PIN);

    if (optSetting & 1) {
        opt0mode::initDataBus();
    }

    Wire.begin();
    Wire.setClock(100000);  // use 100 kHz I2C

    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);

    sensor.setTimeout(500);
    if (!sensor.init()) {
        while (1);
    }
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(100000);
    sensor.startContinuous(100);
}

void loop() {
    if ((optSetting & 1)) {
        if (!digitalRead(PC5)) {
            // Read Mode
            if (digitalRead(LED0) == 0) {
                digitalWrite(LED0, HIGH);  // Indicate active
                sensor.startContinuous(100);
            }
            delay(100);  // Allow some time between readings
            uint32_t avgDistance = opt0mode::GetAveragedDistance(sensor.read());
            if (optSetting & 0b10) {
                // 4" Lens
                if (avgDistance <= (102 + FOCALOffset)) {
                    digitalWrite(PC6, LOW);    // Activate Limit Switch
                    digitalWrite(LED1, HIGH);  // Indicate Triggered
                } else {
                    digitalWrite(PC6, HIGH);  // Tri-State
                    digitalWrite(LED1, LOW);  // Indicate Not Triggered
                }
            } else {
                // 2" Lens
                if (avgDistance <= (51 + FOCALOffset)) {
                    digitalWrite(PC6, LOW);    // Activate Limit Switch
                    digitalWrite(LED1, HIGH);  // Indicate Triggered
                } else {
                    digitalWrite(PC6, HIGH);  // Tri-State
                    digitalWrite(LED1, LOW);  // Indicate Not Triggered
                }
            }

        } else {
            // Halt
            digitalWrite(PC6, HIGH);  // Tri-State
            if (digitalRead(LED0)) {
                digitalWrite(LED0, LOW);
                digitalWrite(LED1, LOW);
                sensor.stopContinuous();
            }
            return;
        }

    } else {
        uint16_t distance = sensor.read();
        if (distance < 200) {
            digitalWrite(LED1, HIGH);
        } else {
            digitalWrite(LED1, LOW);
        }
    }
}