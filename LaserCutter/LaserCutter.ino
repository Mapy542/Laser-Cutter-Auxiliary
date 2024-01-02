#include <Adafruit_VL6180X.h>
#include <Arduino.h>
#include <Wire.h>

/*
All numbers are consistent units.

Distances are in mm.
Times are in ms.
Speeds are in mm/s.
Frequency is in Hz.
*/

class SimpleAuxiliaryFeature {
   private:
    int outputPin;   // pin to output to
    int inputPin;    // pin to read from for state, (switch or button)
    int triggerPin;  // pin to read from for trigger, (Laser Active Signal)

    bool triggerAffected;  // if the trigger activates the feature, false means
                           // trigger pin is ignored
    bool inputAffected;    // if the input activates the feature, false means input
                           // pin is ignored

    bool triggerOutputRelation;  // true means trigger activates output, false
                                 // means trigger deactivates output
    bool inputOutputRelation;    // true means input activates output, false means
                                 // input deactivates output

    bool inputTriggerRelation;  // true means input and trigger must both be
                                // active to activate output, false means only one
                                // must be active this and/or comparison is made
                                // after the output relations are made

    int triggerPinMode;  // mode to set the trigger pin to
    int inputPinMode;    // mode to set the input pin to

    bool invertOutput;  // if the output should be inverted

    bool EstopAffected;  // if the estop affects the feature, false means estop
                         // pin is ignored

   public:
    SimpleAuxiliaryFeature(int outputPin, int inputPin, int triggerPin, bool triggerAffected,
                           bool inputAffected, bool triggerOutputRelation, bool inputOutputRelation,
                           bool inputTriggerRelation, int triggerPinMode, int inputPinMode,
                           bool invertOutput, bool estopAffected) {
        this->outputPin = outputPin;
        this->inputPin = inputPin;
        this->triggerPin = triggerPin;
        this->triggerAffected = triggerAffected;
        this->inputAffected = inputAffected;
        this->triggerOutputRelation = triggerOutputRelation;
        this->inputOutputRelation = inputOutputRelation;
        this->inputTriggerRelation = inputTriggerRelation;
        this->triggerPinMode = triggerPinMode;
        this->inputPinMode = inputPinMode;
        this->invertOutput = invertOutput;
        this->EstopAffected = estopAffected;
    }

    void initialize() {  // called once to initialize the feature
        pinMode(outputPin, OUTPUT);
        pinMode(inputPin, inputPinMode);
        pinMode(triggerPin, triggerPinMode);

        digitalWrite(outputPin, invertOutput);
    }

    void update(bool estop) {  // called every loop to update the output pin

        if (estop && EstopAffected) {
            digitalWrite(outputPin, false != invertOutput);
            return;
        }

        bool triggerActive = digitalRead(triggerPin);
        bool inputActive = digitalRead(inputPin);

        bool outputActive = false;

        bool relatedTrigger = triggerActive == triggerOutputRelation;
        bool relatedInput = inputActive == inputOutputRelation;

        if (!triggerAffected) {
            outputActive = relatedInput != invertOutput;
        } else if (!inputAffected) {
            outputActive = relatedTrigger != invertOutput;
        } else {
            if (inputTriggerRelation) {
                outputActive = (relatedTrigger && relatedInput) != invertOutput;
            } else {
                outputActive = (relatedTrigger || relatedInput) != invertOutput;
            }
        }

        digitalWrite(outputPin, outputActive);
    }
};

SimpleAuxiliaryFeature simpleFeatures[] = {
    SimpleAuxiliaryFeature(52, 35, A0, true, true, true, false, false, INPUT, INPUT_PULLUP, true,
                           true),  // Air Assist
    SimpleAuxiliaryFeature(37, 34, A0, false, true, false, false, false, INPUT, INPUT_PULLUP, false,
                           true),  // Water Chiller
    SimpleAuxiliaryFeature(51, 33, A0, false, true, false, false, false, INPUT, INPUT_PULLUP, true,
                           true),  // Water Pump
    SimpleAuxiliaryFeature(36, 32, A0, true, true, true, false, false, INPUT, INPUT_PULLUP, true,
                           true),  // Exhaust Fan
    // SimpleAuxiliaryFeature(52, A12, A0, false, true, false, false, false, INPUT, INPUT_PULLUP,
    // true,
    //                        false),  // Cabinet LEDS
};

byte simpleFeatureCount = sizeof(simpleFeatures) / sizeof(SimpleAuxiliaryFeature);

///////////////////

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

struct VlData ReadVlSensor() {
    if (!vl.isRangeComplete()) {  // if range is not complete, return empty data
        struct VlData empty;
        empty.range = 0;
        empty.status = 0;
        empty.success = false;
        return empty;
    }

    uint8_t status = vl.readRangeStatus();  // read range status
    uint8_t range = vl.readRangeResult();  // read range result and clear interrupt for next reading
    Serial.println(range);

    VlDebug(status);

    struct VlData returns;
    returns.range = range;
    returns.status = status;
    returns.success = status == VL6180X_ERROR_NONE;

    return returns;
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

   public:
    BedLift(uint8_t dirPin, uint8_t stepPin, bool reverseDirection, uint8_t upLimitSwitchPin,
            uint8_t downLimitSwitchPin, int upLimitSwitchPinMode, int downLimitSwitchPinMode,
            bool invertUpLimitSwitch, bool invertDownLimitSwitch, int upButtonPin,
            int downButtonPin, bool invertUpButton, bool invertDownButton, int upButtonPinMode,
            int downButtonPinMode, int upButtonLEDPin, int downButtonLEDPin,
            int tripleToggleInput1Pin, int tripleToggleInput2Pin, bool invertTripleToggleInput1,
            bool invertTripleToggleInput2, int tripleToggleInput1PinMode,
            int tripleToggleInput2PinMode, uint32_t maxStepFrequency, int focalLength,
            int sensorOffset) {
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

        this->Kp = 0.1;
        this->Ki = 0.0;
        this->Kd = 0.0;
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
        digitalWrite(dirPin, dir != reverseDirection);
        if (speed == 0) {
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

        float speed = Kp * (float)error + Ki * (float)integral + Kd * (float)derivative;

        if (speed > 254) {
            speed = 254;
        } else if (speed < -254) {
            speed = -254;
        }

        uint8_t speedByte = abs(floor(speed));

        if (error ==
            0) {  // if error is 0, stop moving (acceptable because error is whole mm divisions)
            stopActuate();
        } else {
            updateActuate(error > 0, speedByte);
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
            IlluminateButtonLEDs(
                upLimitSwitchActive,
                downLimitSwitchActive);  // illuminate leds based on which directions are available
            if (upButtonActive && !upLimitSwitchActive) {  // if up button is pressed and up limit
                                                           // switch is not active
                updateActuate(true, 255);                  // move up
            } else if (downButtonActive &&
                       !downLimitSwitchActive) {  // if down button is pressed and down limit switch
                                                  // is not active
                updateActuate(false, 255);        // move down
            } else {
                stopActuate();  // stop moving
            }
        } else if (tripleToggleInput2Active ==
                   invertTripleToggleInput2) {  // if switch is right, auto continuous compensation
                                                // mode
            IlluminateButtonLEDs(true, true);   // disable leds

            struct VlData VlReturn = ReadVlSensor(true);  // read vl sensor and get returns

            if (VlReturn.success) {                            // if vl sensor read was successful
                int distance = VlReturn.range + sensorOffset;  // get distance from sensor
                int error = distance - focalLength;            // calculate steps to move
                PidActuate(error);                             // actuate based on error
            } else {
                stopActuate();
            }

        } else {                                // middle single shot mode
            IlluminateButtonLEDs(false, true);  // illuminate up button led (as the activate button)

            struct VlData VlReturn = ReadVlSensor();  // read vl sensor and get returns

            if (VlReturn.success == false) {
                Serial.println("read failed/not ready");
                return;
            }

            if (upButtonActive) {
                if (VlReturn.success) {  // if vl sensor read was successful
                    int distance = VlReturn.range + sensorOffset;  // get distance from sensor
                    int error = distance - focalLength;            // calculate steps to move
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

BedLift autoBed = BedLift(2, 3, false, 26, 27, INPUT_PULLUP, INPUT_PULLUP, true, true, A4, A3, true,
                          true, INPUT_PULLUP, INPUT_PULLUP, 44, 45, A8, A7, true, true,
                          INPUT_PULLUP, INPUT_PULLUP, 1000, 2 * 25.4, 5);

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

    for (int i = 0; i < simpleFeatureCount; i++) {  // initialize all features
        simpleFeatures[i].initialize();
    }

    autoBed.initialize();  // initialize bed lift

    pinMode(EStopPin, INPUT);
    pinMode(LaserStopPin, INPUT);
    pinMode(SafetyRelayResetButtonLED, OUTPUT);
    pinMode(BreakEstopPin, OUTPUT);
    pinMode(24, INPUT);

    if (!vl.begin()) {
        Serial.println("Failed to find sensor");
        while (1) {
            delay(100);
        }
    }

    vl.startRangeContinuous(50);  // 50 ms interval continuous mode
}

void loop() {
    bool estop =
        digitalRead(EStopPin) != true;  // read estop pin and laser stop pin (invert signal)
    bool laserStop = digitalRead(LaserStopPin) != true;
    digitalWrite(LED_BUILTIN, estop);  // set builtin led to estop state

    for (int i = 0; i < simpleFeatureCount; i++) {  // update all features
        simpleFeatures[i].update(estop);
    }

    safetyRelayResetButtonIlluminate(estop,
                                     laserStop);  // update safety relay reset button illumination

    digitalWrite(BreakEstopPin,
                 TriggerEStop);  // set break estop pin to trigger estop if applicable

    autoBed.update(estop);  // update bed lift

    delay(300);
}