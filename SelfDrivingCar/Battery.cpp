#include "Arduino.h"
#include "Battery.h"

Battery::Battery(const int batteryPin, const float voltageDivider, const float vRef, const byte cells, const byte chemistry) {
  pin = batteryPin;
  divider = voltageDivider;
  ref = vRef;
  this->cellCount = cells;
  this->batteryChemistry = chemistry;

  pinMode(pin, INPUT);

  // initialize all 10 readings
  for (byte i = 0; i < READINGS; i++)
    getAverage();
}

float Battery::getAverage() {
  total -= readings[readingsIndex];

  readings[readingsIndex] = analogRead(pin);

  total += readings[readingsIndex];

  if (++readingsIndex >= READINGS)
    readingsIndex = 0;

  return (float) total / readingsIndex;
}

/**
 * Get the external power supply voltage
 */
float Battery::getVoltage(const bool average) {
  float v = (average ? getAverage() : analogRead(pin)) * ref / 1024.0;

  return v / divider;
}

/**
 * Get the estimated percentage of battery power left, assuming a 2S LiPo
 */
float Battery::getGas(const bool average) {
  float v = getVoltage(average) / cellCount;

  if (batteryChemistry == CHEMISTRY_LIPO) {
    if (v >= 3.9)
      return 75 + 25. * (v - 3.9) / (4.2 - 3.9);

    if (v >= 3.75)
      return 50 + 25. * (v - 3.75) / (3.9 - 3.75);

    if (v >= 3.7)
      return 25 + 25. * (v - 3.7) / (3.75 - 3.7);

    if (v >= 3.65)
      return 12.5 + 12.5 * (v - 3.65) / (3.7 - 3.65);

    if (v >= 3.6)
      return 6.25 + 6.25 * (v - 3.6) / (3.65 - 3.6);

    if (v >= 3)
      return 6.25 * (v - 3) / (3.6 - 3.0);
  }

  return 0;
}

