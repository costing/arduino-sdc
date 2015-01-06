#ifndef __ServoExth__
#define __ServoExth__

#include <Servo.h>
#include <inttypes.h>

#ifndef REFRESH_INTERVAL
// Intel Edison library doesn't have this constant
#define REFRESH_INTERVAL 0
#endif

/**
 * Servo extension class, implementing travel limits and estimated time to perform the operation
 */
class ServoExt : public Servo
{
  public:
    /**
     * Constructor, passing the maximum travel angle and the time it takes to reach it
     * @param maxAngle maximum angle that can be asked from this servo
     * @param travelTime time in milliseconds to reach the maximum angle, starting from the middle point
     */
    ServoExt(const int8_t maxAngle, const int8_t travelTime);
    
    /**
     * Set a percentage of travel relative to the center, eg [-100, 100]
     * @param percentage percentage of travel to set
     * @param waitToStabilize whether or not to wait for the expected time of the servo to reach the destination
     */
    void setPercentage(const int8_t percentage, const bool waitToStabilize = false);
    
    /**
     * Set the desired angle relative to the center, eg [-90, 90]. It will automatically apply the constructor-set boundary
     * @param angle desired angle
     * @param waitToStabilize whether or not to wait for the expected time of the servo to reach the destination
     */
    void setAngle(int8_t angle, const bool waitToStabilize = false);
    
    /**
     * Wait long enough for the servo to reach the point of the last operation (called with waitToStabilize = false)
     */
    void waitForAngle();
    
    /**
     * Get the angle last set by one of the above operations
     */
    int8_t getAngle();

  private:
    int8_t maxAngle;
    int8_t travelTime;
    int8_t oldAngle;
    int8_t prevAngle;

    unsigned long toStabilize;
};

#endif

