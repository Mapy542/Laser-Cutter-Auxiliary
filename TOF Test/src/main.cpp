#include <Arduino.h>

/*
This example takes range measurements with the VL53L1X and displays additional
details (status and signal/ambient rates) for each measurement, which can help
you determine whether the sensor is operating normally and the reported range is
valid. The range is in units of mm, and the rates are in units of MCPS (mega
counts per second).
*/

#include <VL53L1X.h>
#include <Wire.h>

VL53L1X sensor;

#define LED0 PC7
#define LED1 PD0

void setup() {
    Wire.begin();
    Wire.setClock(100000);  // use 400 kHz I2C

    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);

    sensor.setTimeout(500);
    if (!sensor.init()) {
        while (1);
    }

    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(100000);

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    sensor.startContinuous(100);

    digitalWrite(LED0, HIGH);
    digitalWrite(LED1, HIGH);
}

void loop() {
    if (sensor.read() > 102) {
        digitalWrite(LED0, HIGH);
        digitalWrite(LED1, LOW);
    } else if (sensor.read() > 100) {
        digitalWrite(LED0, LOW);
        digitalWrite(LED1, HIGH);
    } else {
        digitalWrite(LED0, LOW);
        digitalWrite(LED1, LOW);
    }

    delay(100);
}