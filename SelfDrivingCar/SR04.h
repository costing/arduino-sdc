#ifndef __SR04h__
#define __SR04h__

/**
 * Helper functions for the HC SR04 ultrasound range finder module.
 */
class SR04 {
  public:
    /**
     * Constructor, passing the two required pins. The respective direction (INPUT for echo, OUTPUT for trigger) is set by this code.
     * @param echo input pin, reading the distance from the nearest object
     * @param trig trigger pin, output pin triggering the measurement
     */
    SR04(const int echo, const int trig);

    /**
     * Trigger a measurement and return the distance to the nearest object, if positive.
     * @return distance to the nearest object in cm. Zero or negative is a sign of error.
     * @see setThreshold()
     */
    signed int getRange();

    /**
     * Take <code>count</code> measurements and return the average distance to the nearest object, in cm.
     * Errors from the individual measurements are ignored (range values <= 0).
     * @param count number of measurements
     * @return distance to the nearest object as average of this many measurements
     */
    int getRangeAvg(const byte count);

    /**
     * Set the time threshold (in microseconds) for the reply. The pulse timeout will be set to this value.
     * Moreover the return value of getRange() or getRangeAvg() will be cut to this threshold. Realistically speaking
     * the sensor is not usable for more than 4m or so. As such the default threshold is set to 25000.
     * Important: setting this value doesn't guarantee a response time from the sensor in any way!
     * @param newThreshold new time threshold, in microseconds
     */
    void setThreshold(const unsigned long newThreshold);

  private:
    /**
     * echo pin
     */
    unsigned short int echoPin;

    /**
     * trigger pin
     */
    unsigned short int trigPin;

    /**
     * Time threshold
     */
    unsigned long threshold = 25000;
};

#endif
