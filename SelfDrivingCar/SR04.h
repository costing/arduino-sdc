#ifndef __SR04h__
#define __SR04h__

/**
 * Helper functions for the HC SR04 ultrasound range finder module.
 */
class SR04 {
  public:
    /**
     * Constructor, passing the two required pins. The respective direction (INPUT for echo, OUTPUT for trigger) is set by this code.
     * echo: input pin, reading the distance from the nearest object
     * trig: trigger pin, output pin triggering the measurement
     */
    SR04(const int echo, const int trig);

    /**
     * Trigger a measurement and return the distance to the nearest object
     * @return distance to the nearest object in cm
     */
    int getRange();

    /**
     * Take <code>count</code> measurements and return the average distance to the nearest object, in cm
     */
    int getRangeAvg(byte count);

  private:
    /**
     * echo pin
     */
    int echoPin;

    /**
     * trigger pin
     */
    int trigPin;
};

#endif

