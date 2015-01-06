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

  duration = pulseIn(echoPin, HIGH, 250000);  // assume max. 43m to the nearest object, more is probably just a board error

  return duration / (2 * 29.1);
}

int SR04::getRangeAvg(byte count) {
  long range = 0;

  for (byte i = 0; i < count; i++) {
    range += getRange();
  }

  return (int) (range / count);
}

