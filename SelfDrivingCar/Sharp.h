#ifndef __Sharp_h__
#define __Sharp_h__

#include <inttypes.h>

/**
 * Multi map function from here: http://playground.arduino.cc/Main/MultiMap
 * @param val value to map
 * @param in possible ranges for the input value
 * @param out linear ranges to map the input value to
 * @param size number of values in the in and out arrays
 */
int multiMap(const int val, const int* const _in, const int* const _out, const uint8_t size);

/**
 * Simple wrapper around the Sharp GP2Y0A02YK0F module
 */
class Sharp {
  public:
    /**
     * The constructor gets the input pin for this module
     */
    Sharp(const int pin);
    
    /**
     * Get the range to the nearest object in sight
     * @return the distance in cm
     */
    int getRange();
  
  private:
    int pin;
};

#endif

