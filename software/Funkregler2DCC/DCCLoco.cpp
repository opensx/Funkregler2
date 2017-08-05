/* DCCLoco.cpp
 * contains all details of a selectrix(SX1) loco
 
 * cannot include "EEPROM.h" here:
 * "This seems to be a general problem of the Arduino IDE: 
 * It only recognizes libraries that are included in the 
 * (primary) .ino file.
 * solution: include EEPROM.h only in .ino file "
 */

#include "dccutils.h"
#include "DCCLoco.h"

DCCLoco::DCCLoco() {
	_address = 3;
	_functions = 0;
	_changed = 1;
	_absSpeed = 0;    // positive (=abs(actual v))
  _dir = 0;
  _functionsString = "0 0 0 0 0";
}

DCCLoco::DCCLoco(int16_t a) {
	if ((a > 0) && (a <= MAX_ADDRESS)) { 
		_address = a;
	} else {
		_address = 3;
	}
	_functions = 0;
	_changed = 1;
	_absSpeed = 0;
  _dir = 0;
  _functionsString = "0 0 0 0 0";
}

void DCCLoco::init() {
  _functions = 0;
  _changed = 1;
  _absSpeed = 0;
  _dir = 0;
  _functionsString = "0 0 0 0 0"; 
}

uint8_t DCCLoco::hasChanged() {
	return _changed;
}

void DCCLoco::resetChanged() {
	_changed = 0;
}

void DCCLoco::stop() {
	_absSpeed = 0;
	_changed = 1;
}

int16_t DCCLoco::getSpeed() {
	int16_t s = _absSpeed; // absspeed is always positive
	if (_dir) {
		return -s;
	} else {
		return s;
	}
}

uint16_t DCCLoco::getAbsSpeed() {
  return _absSpeed; // absspeed is always positive

}

uint8_t DCCLoco::toggleFunction(uint8_t i) {
  uint8_t funcI = _functions & (1 << i);
	if (funcI) {
     _functions &= ~(1 << i);
     _functionsString.setCharAt((2*i),'0');
	} else {
 	   _functions |=  (1 << i);
     _functionsString.setCharAt((2*i),'1'); 
  }
}

String DCCLoco::getF0F4String() {
  return _functionsString;
}

uint8_t DCCLoco::getBackward() {
	return _dir;
}

void DCCLoco::setBackward(bool b) {
	if (b == true) {
		_dir=1;
	} else {
		_dir=0;
	}
}

uint16_t DCCLoco::getAddress() {
	return _address;
}

uint16_t DCCLoco::setAddress(uint16_t a) {
	if ((a > 0) && (a <= MAX_ADDRESS)) {
		_address = a;
	} else {
		_address = 3;
	}
	return _address;
}


/** set the speed from number -127 .. +127
 *  
 */
int16_t DCCLoco::setSpeed(int16_t sp) { 
	//Serial.print("DCCLoco.setSpeed, sp=");
	//Serial.println(sp);

	if (sp > MAX_SPEED) {
    sp = MAX_SPEED;
	} else if (sp < -MAX_SPEED) {
    sp = -MAX_SPEED;
	}
		

	uint16_t _newSpeed;
	uint8_t _newDir = _dir;

	// do not change direction when new speed == 0
	if (sp < 0) {
		_newDir = 1;
	} else if (sp > 0) {
		_newDir = 0;
	}
	_newSpeed = (uint16_t)abs(sp);

	// did speed or direction change ?
	if ((_newSpeed != _absSpeed)
			|| (_newDir != _dir)) {
		_absSpeed = _newSpeed;
    _dir = _newDir;
		_changed = 1;
	}
  return sp;
}


/** update speed from rotary decoder setting
 *  
 */
int16_t DCCLoco::updateSpeed(int16_t newSpeed) { 
    int16_t updatedSpeed;
/* have a "zero with sign" i.e. stop the loco first
         without changing the direction, then change the direction
         but leave speed at 0, then normal speed setting */
        if ((newSpeed < 0) && (_dir == 0)) {
          updatedSpeed = 0;
          setSpeed(0);
          _dir = 1;
        } else if ((newSpeed > 0) && (_dir != 0)) {
          updatedSpeed = 0;
          setSpeed(0);
           _dir = 0;
        } else if (newSpeed == 0) {
          updatedSpeed = 0;
          stop();
        } else {
          if ( 
               ((millis() - _modTime) < 100) 
               && (_absSpeed >= 5) 
             ) {
            if (newSpeed > getSpeed()) {
              newSpeed += 10;
              if (newSpeed > MAX_SPEED) newSpeed = MAX_SPEED;
            } else {
              newSpeed -= 10;
              if (newSpeed < -MAX_SPEED) newSpeed = -MAX_SPEED;
            } 
            updatedSpeed = newSpeed;         
          }
                 
          setSpeed(newSpeed);
        }
        _modTime = millis();
        return updatedSpeed;
}
