/* Loco.cpp
 * contains all details of a selectrix(SX1) loco
 
 * cannot include "EEPROM.h" here:
 * "This seems to be a general problem of the Arduino IDE: 
 * It only recognizes libraries that are included in the 
 * (primary) .ino file.
 * solution: include EEPROM.h only in .ino file "
 */

#include "utils.h"
#include "Loco.h"

Loco::Loco() {
	_address = 3;
	_sxData = 0;
	_changed = 1;
	_speed = 0;
}

Loco::Loco(int16_t a) {
	if ((a > 0) && (a <= 103)) {   //TODO must be changed for DCC
		_address = (uint8_t) a;
	} else {
		_address = 3;
	}
	_sxData = 0;
	_changed = 1;
	_speed = 0;
}

uint8_t Loco::hasChanged() {
	return _changed;
}

void Loco::resetChanged() {
	_changed = 0;
}

void Loco::stop() {
	_sxData &= 0xE0; // set speed bits (only) to zero
	_changed = 1;
}

int16_t Loco::getSpeed() {
	int16_t s = _sxData & 0x1F; // positive
	if (bitRead(_sxData, 5)) {
		return -s;
	} else {
		return s;
	}
}

uint8_t Loco::toggleF0() {
	if (bitRead(_sxData, 6)) {
		bitClear(_sxData, 6);
	} else {
		bitSet(_sxData, 6);
	}
	_changed = 1;
	return bitRead(_sxData, 6);
}

uint8_t Loco::getF0() {
	return bitRead(_sxData, 6);
}

uint8_t Loco::toggleF1() {
	if (bitRead(_sxData, 7)) {
		bitClear(_sxData, 7);
	} else {
		bitSet(_sxData, 7);
	}
	_changed = 1;
	return bitRead(_sxData, 7);
}

uint8_t Loco::toggleFunction(uint8_t i) {
	if (i == 0) {
		return toggleF0();
	} else if (i == 1) {
		return toggleF1();
	} else {
		return 0;   // there are no functions F2, F3, F4 with Selectrix
	}
}

uint8_t Loco::getF1() {
	return bitRead(_sxData, 7);
}

uint8_t Loco::getBackward() {
	return bitRead(_sxData, 5);
}

void Loco::setBackward(bool b) {
	if (b == true) {
		bitSet(_sxData, 5);
	} else {
		bitClear(_sxData, 5);
	}
}

uint16_t Loco::getAddress() {
	return _address;
}

uint16_t Loco::setAddress(uint16_t a) {
	if ((a > 0) && (a <= 103)) {
		_address = (uint8_t) a;
	} else {
		_address = 3;
	}
	return _address;
}

uint8_t Loco::getSXData() {
	// sx data (bit 5=direction, bit 6= light, bit7= horn)
	return _sxData;
}

void Loco::setFromSXData(uint8_t data) {
	// sx data (bit 5=direction, bit 6= light, bit7= horn)
	_sxData = data;
}

/** set the speed from number -31 .. +31
 *  
 */
void Loco::setSpeed(int16_t sp) {  //TODO must be changed for DCC
	//Serial.print("SXLoco.setSpeed, sp=");
	//Serial.println(sp);

	if (sp > 31)
		sp = 31;
	if (sp < -31)
		sp = -31;

	uint8_t _newSpeed;
	uint8_t _newBackward = 0;

	// do not change direction when new speed == 0
	if (sp < 0) {
		_newBackward = 1;
	} else if (sp > 0) {
		_newBackward = 0;
	}
	_newSpeed = abs(sp);

	// did speed or direction change ?
	if ((_newSpeed != (_sxData & 0x1F))
			|| (_newBackward != bitRead(_sxData, 5))) {
		_sxData = (_sxData & 0xE0) | _newSpeed;
		if (_newBackward) {
			bitSet(_sxData, 5);
		} else {
			bitClear(_sxData, 5);
		}
		_changed = 1;
	}

}
