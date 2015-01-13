#include <Arduino.h>
#include "SR04.h"

SR04::SR04(const int echo, const int trig) {
  this->echoPin = echo;
  this->trigPin = trig;

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // set the trigger level to LOW
  digitalWrite(trigPin, LOW);
}

int SR04::getRange() {
  long duration;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, threshold);

  if (duration > threshold)
    return 0;

  return duration / 58;
}

int SR04::getRangeAvg(const byte count) {
  long range = 0;
  byte cnt = 0;

  for (byte i = 0; i < count; i++) {
    const long newRange = getRange();

    if (newRange > 0) {
      range += newRange;
      cnt = 0;
    }
  }

  if (cnt == 0)
    return 0;

  return (int) (range / cnt);
}

void SR04::setThreshold(const unsigned long newThreshold) {
  this->threshold = newThreshold;
}

