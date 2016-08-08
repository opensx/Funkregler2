// -----
// RotaryEncoderMax.cpp - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// added "MAX" functionality : Michael Blank
// -----

#include "Arduino.h"
#include "RotaryEncoderMax.h"


// The array holds the values Ã¯Â¿Â½1 for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.

const int8_t KNOBDIR[] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
  -1,  0,  0,  1,
0,  1, -1,  0  };


// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down


// ----- Initialization and Default Values -----

RotaryEncoderMax::RotaryEncoderMax(int pin1, int pin2) {
  
  // Remember Hardware Setup
  _pin1 = pin1;
  _pin2 = pin2;
  
  _oldState = 3;

  // start with position 0;
  _position = 0;
  _positionExt = 0;

} // RotaryEncoderMax()

void RotaryEncoderMax::init() {
  // for MKR1000
  pinMode(_pin1, INPUT_PULLUP);  // turn on pullup resistor 
  pinMode(_pin2, INPUT_PULLUP);  // turn on pullup resistor
  
}

int  RotaryEncoderMax::getPosition() {
  return _positionExt;
}


int RotaryEncoderMax::getPositionMax(int max) {
  int _newPos = _positionExt;
  // limit to -MAX ... +MAX
  if (_newPos > max) {
    setPosition(max);
    return max;
  }
  if (_newPos < -max) {
    setPosition(-max);
    return -max;
  }
  return _newPos;
}

void RotaryEncoderMax::setPosition(int newPosition) {
  // only adjust the external part of the position.
  
  _position = ((newPosition<<2) | (_position & 0x03));
  _positionExt = newPosition;

}

void RotaryEncoderMax::tick(void)
{
  int sig1 = digitalRead(_pin1);
  int sig2 = digitalRead(_pin2);
  int8_t thisState = sig1 | (sig2 << 1);

  if (_oldState != thisState) {
    _position += KNOBDIR[thisState | (_oldState<<2)];
    
    if (thisState == LATCHSTATE)
      _positionExt = _position >> 2;
    
    _oldState = thisState;
  } // if
} // tick()

// End


