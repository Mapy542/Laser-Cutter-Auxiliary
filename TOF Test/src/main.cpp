#include "Adafruit_VL53L1X.h"

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

#ifdef __CH32V__
#define LED0 PC7
#define LED1 PD0
#else
#define LED0 LED_BUILTIN
#define LED1 LED_BUILTIN
#endif

void setup() {
    Wire.begin();
    if (!vl53.begin(0x29, &Wire)) {
        while (1) delay(10);
    }
    if (!vl53.startRanging()) {
        while (1) delay(10);
    }

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    vl53.VL53L1X_SetDistanceMode(1);
    vl53.setTimingBudget(50);

    /*
    vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
    vl.VL53L1X_SetInterruptPolarity(0);
    */

    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);
}

void loop() {
    int16_t distance;

    if (vl53.dataReady()) {
        // new measurement for the taking!
        distance = vl53.distance();
        if (distance == -1) {
            return;
        }
        if (distance > 102) {
            digitalWrite(LED0, HIGH);
            digitalWrite(LED1, LOW);
        } else if (distance > 100) {
            digitalWrite(LED0, LOW);
            digitalWrite(LED1, HIGH);
        } else {
            digitalWrite(LED0, LOW);
            digitalWrite(LED1, LOW);
        }

        // data is read out, time for another reading!
        vl53.clearInterrupt();
    }
}