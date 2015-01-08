#include "ServoExt.h"

#include "Arduino.h"

ServoExt::ServoExt(const unsigned short int maxAngle, const unsigned short int travelTime) {
  this->maxAngle = maxAngle;
  this->travelTime = travelTime;
  this->toStabilize = 0;
  this->prevAngle = this->oldAngle = 90;
}

void ServoExt::setAngle(unsigned short int angle, const bool waitToStabilize) {
  angle = max(angle, 90 - this->maxAngle);
  angle = min(angle, 90 + this->maxAngle);

  if (angle == prevAngle) {
    if (waitToStabilize)
      waitForAngle();

    return;
  }

  write(angle);

  if (toStabilize > millis()) {
    const int yetToReach = (toStabilize - millis()) * this->maxAngle / this->travelTime;

    if (oldAngle < prevAngle)
      prevAngle -= yetToReach;
    else
      prevAngle += yetToReach;
  }

  const int diff = angle > prevAngle ? angle - prevAngle : prevAngle - angle;

  const long wait = REFRESH_INTERVAL / 1000 + (diff * this->travelTime) / this->maxAngle;

  oldAngle = prevAngle;
  prevAngle = angle;

  if (waitToStabilize) {
    delay(wait);
    toStabilize = millis();
  }
  else {
    toStabilize = millis() + wait;
  }
}

void ServoExt::setPercentage(const unsigned short int percentage, const bool waitToStabilize) {
  setAngle(90 + percentage * maxAngle / 100, waitToStabilize);
}

void ServoExt::waitForAngle() {
  if (toStabilize < millis())
    delay(millis() - toStabilize);
}

unsigned short int ServoExt::getAngle() {
  return this->prevAngle;
}
