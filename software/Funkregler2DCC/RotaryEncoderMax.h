// -----
// RotaryEncoder.h - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 23.07.2106 modified (max) by michael blank
// -----

#ifndef RotaryEncoderMax_h
#define RotaryEncoderMax_h

#include "Arduino.h"

#define LATCHSTATE 3

class RotaryEncoderMax
{
public:
  // ----- Constructor -----
  RotaryEncoderMax(int pin1, int pin2);

  // initialize arduino pins
  void init(void);

  // retrieve the current position
  int  getPosition();
  int  getPositionMax(int max);

  // adjust the current position
  void setPosition(int newPosition);

  // call this function every some milliseconds or by using an interrupt for handling state changes of the rotary encoder.
  void tick(void);

private:
  int _pin1, _pin2; // Arduino pins used for the encoder.

  int8_t _oldState;

  int _position;     // Internal position (4 times _positionExt)
  int _positionExt;  // External position
};

#endif

// End


