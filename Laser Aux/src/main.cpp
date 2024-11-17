#include <Arduino.h>

/*
Laser Control Program for K400 Laser Cutter.
Arduino Mega 2560

Created by: Eli Bukoski

Arduino controls auxiliary features and bed lift. M2 Nano controls laser and laser power supply.

Auxiliary Features:
    Air Assist
    Water Chiller
    Water Pump
    Exhaust Fan
    Laser Manual Fire
    Cabinet LEDS

Bed Lift:
    Auto Continuous Compensation
    Manual
    Single Shot

Work in Progress: Bed Lift PID
    The control algorithm is basic and choppy. Averaging sensor values would greatly improve the
output of the PID loop. VL6180X sensor is disappointingly inaccurate (+- 4 mm). However, it is the
only commodity sensor with a compatible range.
*/

/*
All numbers are consistent units.

Distances are in mm.
Times are in ms.
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

    SimpleAuxiliaryFeature(42, 31, A0, false, true, false, false, false, INPUT, INPUT_PULLUP, true,
                           true),  // Laser Manual Fire
    // SimpleAuxiliaryFeature(52, A12, A0, false, true, false, false, false, INPUT, INPUT_PULLUP,
    // true,
    //                        false),  // Cabinet LEDS
};

byte simpleFeatureCount = sizeof(simpleFeatures) / sizeof(SimpleAuxiliaryFeature);

///////////////////

void (*resetFunction)(void) = 0;  // declare reset function @ address 0

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
        } else {
            stopActuate();
        }
    }
};

BedLift autoBed = BedLift(2, 3, false, 26, 27, INPUT_PULLUP, INPUT_PULLUP, true, true, A4, A3, true,
                          true, INPUT_PULLUP, INPUT_PULLUP, 44, 45, A8, A7, true, true,
                          INPUT_PULLUP, INPUT_PULLUP, 15000, 2 * 25.4, 15);

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

    delay(30);
}