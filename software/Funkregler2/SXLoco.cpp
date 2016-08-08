/* SXLoco.cpp
 * contains all details of a selectrix(SX1) loco
 
 * cannot include "EEPROM.h" here:
 * "This seems to be a general problem of the Arduino IDE: 
 * It only recognizes libraries that are included in the 
 * (primary) .ino file.
 * solution: include EEPROM.h only in .ino file "
 */

#include "sxutils.h"
#include "SXLoco.h"

SXLoco::SXLoco() {
    _address = 3;
    _sxData = 0;
    _changed= 1;
}

SXLoco::SXLoco(int a) {
	if ((a > 0) && (a <= 103)) {
      _address = (uint8_t) a;
} else {
	_address = 3;
}
    _sxData = 0;
    _changed= 1;
}

uint8_t SXLoco::hasChanged() {
   return _changed;
}

void SXLoco::resetChanged() {
   _changed = 0;
}

void SXLoco::stop() {
    _sxData &=0xE0; // set speed bits (only) to zero
    _changed=1;
}

int SXLoco::getSpeed() {
    int s = _sxData & 0x1F; // positive
    if (bitRead(_sxData,5)) {
      return -s;
    } else {
      return s;
    }
}

void SXLoco::toggleF0() {
   if (bitRead(_sxData,6)) {
      bitClear(_sxData,6);
   } else {
      bitSet(_sxData,6);
   }
   _changed=1;
}

uint8_t SXLoco::getF0() {
   return bitRead(_sxData,6);
}

void SXLoco::toggleF1() {
   if (bitRead(_sxData,7)) {
      bitClear(_sxData,7);
   } else {
      bitSet(_sxData,7);
   }
   _changed=1;
}

uint8_t SXLoco::getF1() {
   return bitRead(_sxData,7);
}

uint8_t SXLoco::getBackward() {
    return bitRead(_sxData,5);
}

void SXLoco::setBackward(bool b) {
  if (b == true) {
    bitSet(_sxData,5);
  } else {
    bitClear(_sxData,5);
  }
}

int SXLoco::getAddress() {
    return _address;
}


int SXLoco::setAddress(int a) {
	if ((a > 0) && (a <= 103)) {
      _address = (uint8_t) a;
} else {
	_address = 3;
}
	 return _address;
}

uint8_t SXLoco::getSXData() {
   // sx data (bit 5=direction, bit 6= light, bit7= horn)
   return _sxData;
}

void SXLoco::setFromSXData(uint8_t data) {
   // sx data (bit 5=direction, bit 6= light, bit7= horn)
   _sxData = data;
}


/** set the speed from number -31 .. +31
 *  
 */
void SXLoco::setSpeed(int sp) {
   //Serial.print("SXLoco.setSpeed, sp=");
   //Serial.println(sp);
   
   if (sp > 31) sp = 31;
   if (sp < -31) sp = -31;

   uint8_t _newSpeed;
   uint8_t _newBackward=0;

   // do not change direction when new speed == 0
   if (sp < 0) {
      _newBackward = 1;
   } else if (sp > 0) {
      _newBackward = 0;
   }
   _newSpeed = abs(sp); 


   // did speed or direction change ?
   if ( (_newSpeed != (_sxData & 0x1F)) ||
        (_newBackward != bitRead(_sxData,5)) ) {
       _sxData = (_sxData & 0xE0) | _newSpeed;
       if (_newBackward) {
           bitSet(_sxData,5);
       } else {
           bitClear(_sxData,5);
       }
       _changed=1;   
   }
   
}




