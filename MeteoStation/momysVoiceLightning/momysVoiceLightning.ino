#include <Arduino.h>
#include <math.h>
#include "DFRobot_AS3935_I2C.h"

volatile int8_t AS3935IsrTrig = 0;

#if defined(ESP32) || defined(ESP8266)
#define IRQ_PIN       0
#else
#define IRQ_PIN       2
#endif

#define AS3935_CAPACITANCE   96
#define AS3935_INDOORS       0
#define AS3935_OUTDOORS      1
#define AS3935_MODE          AS3935_INDOORS
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1
#define AS3935_DIST          AS3935_DIST_EN
#define AS3935_I2C_ADDR      AS3935_ADD3

DFRobot_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)AS3935_I2C_ADDR);

// Function to convert 21-bit digital value to analog voltage
double convertToAnalog(int digitalValue, double Vmin, double Vmax) {
    return Vmin + ((Vmax - Vmin) * digitalValue) / ((1 << 21) - 1);
}

// Function to calibrate analog signal to intensity
double calibrateIntensity(double analogVoltage, double slope, double intercept) {
    return slope * analogVoltage + intercept;
}

// Function to compute distance attenuation factor
double distanceAttenuation(double distance) {
    return 1 / pow(distance, 2); // Using inverse square law
}

// Function to estimate intensity after distance attenuation
double estimateIntensity(double calibratedIntensity, double attenuationFactor) {
    return calibratedIntensity * attenuationFactor;
}

// Function to compute VA rating
double computeVARating(double voltage, double estimatedIntensity) {
    return voltage * estimatedIntensity;
}

void AS3935_ISR() {
    AS3935IsrTrig = 1;
}

void setup() {
    Serial.begin(115200);
    Serial.println("DFRobot AS3935 lightning sensor begin!");

    while (lightning0.begin() != 0){
        Serial.print(".");
    }
    lightning0.defInit();

#if defined(ESP32) || defined(ESP8266)
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN),AS3935_ISR,RISING);
#else
    attachInterrupt(/*Interrupt No*/0,AS3935_ISR,RISING);
#endif

    lightning0.manualCal(AS3935_CAPACITANCE, AS3935_MODE, AS3935_DIST);
}

void loop() {
    while (AS3935IsrTrig == 0) { delay(1); }
    delay(5);

    AS3935IsrTrig = 0;

    uint8_t intSrc = lightning0.getInterruptSrc();
    if (intSrc == 1) {
        uint8_t lightningDistKm = lightning0.getLightningDistKm();
        Serial.println("Lightning occurs!");
        Serial.print("Distance: ");
        Serial.print(lightningDistKm);
        Serial.println(" km");

        uint32_t lightningEnergyVal = lightning0.getStrikeEnergyRaw();

        // Initialization values
        int digitalValue = lightningEnergyVal;
        double Vmin = 0.0;
        double Vmax = 5.0;
        double slope = 0.1;
        double intercept = 0.0;
        double distance = lightningDistKm;
        double voltage = 0.0;
        //computed values
        double analogVoltage = convertToAnalog(digitalValue, Vmin, Vmax);
        double calibratedIntensity = calibrateIntensity(analogVoltage, slope, intercept);
        double attenuationFactor = distanceAttenuation(distance);
        double estimatedIntensity = estimateIntensity(calibratedIntensity, attenuationFactor);
        double VA_rating = computeVARating(voltage, estimatedIntensity);

        Serial.print("Intensity: ");
        Serial.print(estimatedIntensity);
        Serial.println(" VA");
        Serial.print("VA Rating: ");
        Serial.print(VA_rating);
        Serial.println(" VA");
    }
    else if (intSrc == 2) {
        Serial.println("Disturber discovered!");
    }
    else if (intSrc == 3) {
        Serial.println("Noise level too high!");
    }
}
