#ifndef __Batteryh__
#define __Batteryh__

#define CHEMISTRY_LIPO 1
#define CHEMISTRY_NIMH 2

#define DEFAULT_CHEMISTRY CHEMISTRY_LIPO
#define DEFAULT_CELL_COUNT 2
#define DEFAULT_VREF 4.93

#define READINGS 10

#include <Arduino.h>

/**
 * Battery gas estimation utility
 * Connect an analog input pin to the middle of a voltage divider.
 *     Vbat -- R1 -- analog pin -- R2 -- Gnd (common between Arduino and the battery)
 *                       ^
 *
 * The point is to make _sure_ the pin is _never_ above 5V.
 * Second argument to the constructor is the voltage divider ratio, ie. R2 / (R1+R2). If you measure a single cell this value should be 1.
 * Other typical examples: R1 = 10Kohm, R2 = 1Kohm.
 */
class Battery {
  public:
    /**
     * Constructor
     * @param batteryPin the _analog_ pin to read the voltage from, will be set to INPUT by the constructor
     * @param voltageDivider value of R2 / (R1+R2)
     * @param vRef reference voltage, the measured 5V pin on the board
     * @param cells number of cells in the battery pack
     * @param chemisty one of CHEMISTY_LIPO or CHEMISTRY_NIMH constants
     */
    Battery(const int batteryPin, const float voltageDivider, const float vRef = DEFAULT_VREF, const byte cells = DEFAULT_CELL_COUNT, const byte chemistry = DEFAULT_CHEMISTRY);

    /**
     * Get the voltage, either instantaneous or the running average of the last READINGS values
     * @param average if <code>true</code> then return the average, if <code>false</code> then just return the current value
     * @return battery voltage, taking into account the voltage divider and the reference voltage value
     */
    float getVoltage(const bool average);

    /**
     * Get the estimated percentage of battery full, function of the voltage, number of cells and chemistry
     * @param average same as for the getVoltage() method
     * @return percentage of battery full
     */
    float getGas(const bool average);
  private:
    float getAverage();
    int pin;
    float divider;
    float ref;
    byte cellCount;
    byte batteryChemistry;
    int readings[READINGS];
    byte readingsIndex;
    int total;
};

#endif
