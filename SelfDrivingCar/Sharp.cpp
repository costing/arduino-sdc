#include "Sharp.h"

#include <Arduino.h>

int multiMap(const int val, const int* const _in, const int* const _out, const uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

const int irOut[] = {150,140,130,120,110,100, 90, 80, 70, 60, 50, 40, 30, 20};
const int irIn[]  = { 90, 97,105,113,124,134,147,164,185,218,255,317,408,506};

Sharp::Sharp(const int pin){
  this->pin = pin;
}

int Sharp::getRange(){
  return multiMap(analogRead(pin), irIn, irOut, 14);
}
