// #include <AccelStepper.h>
#include <Adafruit_VL6180X.h>
#include <Arduino.h>
#include <SPI.h>
#include <SingleEMAFilterLib.h>
#include <Wire.h>

/*
Laser Control Program for K400 Laser Cutter.
Arduino Mega 2560

Created by: Eli Bukoski
*/

/*
All numbers are consistent units.

Distances are in mm.
Times are in ms.
Frequency is in Hz.
*/

int readFailCount = 0;        // number of times the vl sensor has failed to read
int failResetThreshold = 30;  // number of times the vl sensor can fail to read before it is reset

struct VlData {
    uint8_t range;
    uint8_t status;
    bool success;
};

Adafruit_VL6180X vl = Adafruit_VL6180X();

void VlDebug(uint8_t status) {
    if ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
        Serial.println("System error");
    } else if (status == VL6180X_ERROR_ECEFAIL) {
        Serial.println("ECE failure");
    } else if (status == VL6180X_ERROR_NOCONVERGE) {
        Serial.println("No convergence");
    } else if (status == VL6180X_ERROR_RANGEIGNORE) {
        Serial.println("Ignoring range");
    } else if (status == VL6180X_ERROR_SNR) {
        Serial.println("Signal/Noise error");
    } else if (status == VL6180X_ERROR_RAWUFLOW) {
        Serial.println("Raw reading underflow");
    } else if (status == VL6180X_ERROR_RAWOFLOW) {
        Serial.println("Raw reading overflow");
    } else if (status == VL6180X_ERROR_RANGEUFLOW) {
        Serial.println("Range reading underflow");
    } else if (status == VL6180X_ERROR_RANGEOFLOW) {
        Serial.println("Range reading overflow");
    }
}

/*!
    @brief Reads the vl sensor and returns the range and status

    @returns VlData struct containing the range, status, and success of the reading. If reading is
    isRangeComplete, success will be false and range and status will be 0.
*/

VlData ReadVlSensor() {
    if (!vl.isRangeComplete()) {  // if range is not complete, return empty data
        VlData empty;
        empty.range = 0;
        empty.status = 0;
        empty.success = false;
        return empty;
    }

    uint8_t status = vl.readRangeStatus();  // read range status
    uint8_t range = vl.readRangeResult();  // read range result and clear interrupt for next reading
    Serial.println(range);

    VlDebug(status);

    VlData returns;
    returns.range = range;
    returns.status = status;
    returns.success = status == VL6180X_ERROR_NONE;

    return returns;
}

void (*resetFunction)(void) = 0;  // declare reset function @ address 0

void CheckReset() {
    return;  // disable reset
    if (readFailCount > failResetThreshold) {
        // Serial.println("Resetting VL Sensor");
        resetFunction();  // call reset
    }
}

///////////////////

class BedLift {
   private:
    uint8_t dirPin;
    uint8_t stepPin;

    bool reverseDirection;

    uint8_t upLimitSwitchPin;
    uint8_t downLimitSwitchPin;

    int upLimitSwitchPinMode;
    int downLimitSwitchPinMode;

    bool invertUpLimitSwitch;
    bool invertDownLimitSwitch;

    int upButtonPin;
    int downButtonPin;

    int upButtonPinMode;
    int downButtonPinMode;

    bool invertUpButton;
    bool invertDownButton;

    int upButtonLEDPin;
    int downButtonLEDPin;

    int tripleToggleInput1Pin;
    int tripleToggleInput2Pin;

    bool invertTripleToggleInput1;
    bool invertTripleToggleInput2;

    int tripleToggleInput1PinMode;
    int tripleToggleInput2PinMode;

    uint32_t maxStepFrequency;

    int focalLength;
    int sensorOffset;

    float Kp;
    float Ki;
    float Kd;

    int lastError;
    int integral;

    int *averagingArray;
    int averagingArrayLength;

    SingleEMAFilter<uint8_t> filter;

   public:
    BedLift(uint8_t dirPin, uint8_t stepPin, bool reverseDirection, uint8_t upLimitSwitchPin,
            uint8_t downLimitSwitchPin, int upLimitSwitchPinMode, int downLimitSwitchPinMode,
            bool invertUpLimitSwitch, bool invertDownLimitSwitch, int upButtonPin,
            int downButtonPin, bool invertUpButton, bool invertDownButton, int upButtonPinMode,
            int downButtonPinMode, int upButtonLEDPin, int downButtonLEDPin,
            int tripleToggleInput1Pin, int tripleToggleInput2Pin, bool invertTripleToggleInput1,
            bool invertTripleToggleInput2, int tripleToggleInput1PinMode,
            int tripleToggleInput2PinMode, uint32_t maxStepFrequency, int focalLength,
            int sensorOffset, float filterAlpha)
        : filter(filterAlpha) {
        this->dirPin = dirPin;
        this->stepPin = stepPin;
        this->reverseDirection = reverseDirection;
        this->upLimitSwitchPin = upLimitSwitchPin;
        this->downLimitSwitchPin = downLimitSwitchPin;
        this->upLimitSwitchPinMode = upLimitSwitchPinMode;
        this->invertUpLimitSwitch = invertUpLimitSwitch;
        this->invertDownLimitSwitch = invertDownLimitSwitch;
        this->downLimitSwitchPinMode = downLimitSwitchPinMode;
        this->upButtonPin = upButtonPin;
        this->downButtonPin = downButtonPin;
        this->upButtonPinMode = upButtonPinMode;
        this->downButtonPinMode = downButtonPinMode;
        this->invertUpButton = invertUpButton;
        this->invertDownButton = invertDownButton;
        this->upButtonLEDPin = upButtonLEDPin;
        this->downButtonLEDPin = downButtonLEDPin;
        this->tripleToggleInput1Pin = tripleToggleInput1Pin;
        this->tripleToggleInput2Pin = tripleToggleInput2Pin;
        this->tripleToggleInput1PinMode = tripleToggleInput1PinMode;
        this->tripleToggleInput2PinMode = tripleToggleInput2PinMode;
        this->invertTripleToggleInput1 = invertTripleToggleInput1;
        this->invertTripleToggleInput2 = invertTripleToggleInput2;
        this->maxStepFrequency = maxStepFrequency;
        this->focalLength = focalLength;
        this->sensorOffset = sensorOffset;
        this->averagingArrayLength = sizeof(averagingArray) / sizeof(int);

        this->Kp = 30;
        this->Ki = 4;
        this->Kd = 0;  // should not use because of noise
        this->lastError = 0;
        this->integral = 0;
    }

    void initialize() {
        pinMode(dirPin, OUTPUT);
        pinMode(stepPin, OUTPUT);
        pinMode(upLimitSwitchPin, upLimitSwitchPinMode);
        pinMode(downLimitSwitchPin, downLimitSwitchPinMode);
        pinMode(upButtonPin, upButtonPinMode);
        pinMode(downButtonPin, downButtonPinMode);
        pinMode(upButtonLEDPin, OUTPUT);
        pinMode(downButtonLEDPin, OUTPUT);
        pinMode(tripleToggleInput1Pin, tripleToggleInput1PinMode);
        pinMode(tripleToggleInput2Pin, tripleToggleInput2PinMode);
    }

    void updateActuate(bool dir, uint8_t speed) {
        bool upLimActive = digitalRead(upLimitSwitchPin) != invertUpLimitSwitch;
        bool downLimActive = digitalRead(downLimitSwitchPin) != invertDownLimitSwitch;

        digitalWrite(dirPin, dir != reverseDirection);
        if (speed == 0) {
            noTone(stepPin);
            return;
        } else if (upLimActive && dir) {
            noTone(stepPin);
            return;
        } else if (downLimActive && !dir) {
            noTone(stepPin);
            return;
        } else {
            float frequency = (float)speed / 255.0 * (float)maxStepFrequency;
            int intFrequency = floor(frequency);
            tone(stepPin, intFrequency);
        }
    }

    void stopActuate() { noTone(stepPin); }

    void IlluminateButtonLEDs(bool upButtonActive, bool downButtonActive) {
        digitalWrite(upButtonLEDPin, upButtonActive);
        digitalWrite(downButtonLEDPin, downButtonActive);
    }

    void PidActuate(int error) {  ////////////////////////////////////////////////////////////
        int derivative = error - lastError;
        integral += error;
        lastError = error;

        if (integral > 50) {
            integral = 50;
        } else if (integral < -50) {
            integral = -50;
        }

        // Serial.print("error: ");
        // Serial.print(error);
        // Serial.print(" integral: ");
        // Serial.print(integral);
        // Serial.print(" derivative: ");
        // Serial.print(derivative);
        // Serial.print(" speed: ");

        float speed = Kp * (float)error + Ki * (float)integral + Kd * (float)derivative;

        // Serial.println(speed);

        if (speed > 254) {
            speed = 254;
        } else if (speed < -254) {
            speed = -254;
        } else if (speed < 40 && speed > 0) {
            speed = 40;
        } else if (speed > -40 && speed < 0) {
            speed = -40;
        }

        uint8_t speedByte = abs(floor(speed));

        if (abs(error) <=
            1) {  // if error is 0, stop moving (acceptable because error is whole mm divisions)
            stopActuate();
        } else {
            updateActuate(speed > 0, speedByte);
        }
    }

    void update(bool estop) {
        if (estop) {
            stopActuate();
            return;
        }

        bool upLimitSwitchActive = digitalRead(upLimitSwitchPin) != invertUpLimitSwitch;
        bool downLimitSwitchActive = digitalRead(downLimitSwitchPin) != invertDownLimitSwitch;

        bool upButtonActive = digitalRead(upButtonPin) != invertUpButton;
        bool downButtonActive = digitalRead(downButtonPin) != invertDownButton;

        bool tripleToggleInput1Active =
            digitalRead(tripleToggleInput1Pin) != invertTripleToggleInput1;
        bool tripleToggleInput2Active =
            digitalRead(tripleToggleInput2Pin) != invertTripleToggleInput2;

        if (tripleToggleInput1Active ==
            invertTripleToggleInput1) {  // if switch is left, manual mode
            IlluminateButtonLEDs(upLimitSwitchActive,
                                 downLimitSwitchActive);   // illuminate leds based on which
                                                           // directions are available
            if (upButtonActive && !upLimitSwitchActive) {  // if up button is pressed and up
                                                           // limit switch is not active
                updateActuate(true, 255);                  // move up
            } else if (downButtonActive &&
                       !downLimitSwitchActive) {  // if down button is pressed and down limit
                                                  // switch is not active
                updateActuate(false, 255);        // move down
            } else {
                stopActuate();  // stop moving
            }
        } else if (tripleToggleInput2Active ==
                   invertTripleToggleInput2) {  // if switch is right, auto continuous
                                                // compensation mode
            IlluminateButtonLEDs(true, true);   // disable leds

            struct VlData VlReturn = ReadVlSensor();  // read vl sensor and get returns

            if (!VlReturn.success) {
                // Serial.println("read failed/not ready");
                readFailCount++;
                return;
            }

            if (VlReturn.success) {               // if vl sensor read was successful
                filter.AddValue(VlReturn.range);  // get distance from sensor
                int distance = filter.GetLowPass() - sensorOffset;  // get distance from sensor
                int error = distance - focalLength;                 // calculate steps to move
                // Serial.println(error);
                PidActuate(error);
            }  // actuate based on error
            else {
                stopActuate();  // if vl sensor read was not successful, do not actuate
            }

        } else {  // middle single shot mode
            IlluminateButtonLEDs(false,
                                 true);  // illuminate up button led (as the activate button)

            struct VlData VlReturn = ReadVlSensor();  // read vl sensor and get returns

            if (!VlReturn.success) {
                // Serial.println("read failed/not ready");
                readFailCount++;
                return;
            }

            if (upButtonActive) {
                if (VlReturn.success) {               // if vl sensor read was successful
                    filter.AddValue(VlReturn.range);  // get distance from sensor
                    int distance = filter.GetLowPass() - sensorOffset;  // get distance from sensor
                    int error = distance - focalLength;                 // calculate steps to move
                    // Serial.println(error);
                    PidActuate(error);
                }  // actuate based on error
                else {
                    return;  // if vl sensor read was not successful, do not actuate
                    // may want to add a timeout to prevent getting stuck here
                }
            } else {
                stopActuate();
            }
        }
    }
};

BedLift autoBed = BedLift(2, 3, false, 26, 27, INPUT_PULLUP, INPUT_PULLUP, true, true, A2, A3, true,
                          true, INPUT_PULLUP, INPUT_PULLUP, 35, 34, A0, A1, true, true,
                          INPUT_PULLUP, INPUT_PULLUP, 15000, 2 * 25.4, 15, 0.7);

///////////////////

bool TriggerEStop = false;

int EStopPin = 22;
int LaserStopPin = 23;
int SafetyRelayResetButtonLED = 4;
int BreakEstopPin = 53;

byte safetyRelayResetButtonIlluminationState = 0;

void safetyRelayResetButtonIlluminate(bool estop, bool laserStop) {
    if (estop) {                                        // if estop is not made flash LEDS
        safetyRelayResetButtonIlluminationState += 10;  // increment state
        analogWrite(SafetyRelayResetButtonLED,
                    abs(safetyRelayResetButtonIlluminationState - 128) * 2);  // set LEDS to state
    } else if (laserStop) {                           // if estop is made but laser is not active
        analogWrite(SafetyRelayResetButtonLED, 255);  // turn off LEDS
    } else {                                          // if estop is made and laser is active
        analogWrite(SafetyRelayResetButtonLED, 0);    // turn on LEDS
    }
}

void setup() {
    Serial.begin(115200);

    autoBed.initialize();  // initialize bed lift

    pinMode(EStopPin, INPUT);
    pinMode(LaserStopPin, INPUT);
    pinMode(SafetyRelayResetButtonLED, OUTPUT);
    pinMode(BreakEstopPin, OUTPUT);
    pinMode(24, INPUT);

    if (!vl.begin()) {
        Serial.println("Failed to find sensor");
        delay(1000);      // wait 1 second
        resetFunction();  // call reset
    }

    // vl.i2c_dev.setSpeed(20000);

    vl.startRangeContinuous(20);  // 50 ms interval continuous mode
}

void loop() {
    bool estop =
        digitalRead(EStopPin) != true;  // read estop pin and laser stop pin (invert signal)
    bool laserStop = digitalRead(LaserStopPin) != true;
    digitalWrite(LED_BUILTIN, laserStop);  // set builtin led to estop state

    safetyRelayResetButtonIlluminate(estop,
                                     laserStop);  // update safety relay reset button illumination

    digitalWrite(BreakEstopPin,
                 TriggerEStop);  // set break estop pin to trigger estop if applicable

    autoBed.update(estop);  // update bed lift

    CheckReset();  // check if vl sensor needs to be reset (resets whole arduino)

    delay(3);
}