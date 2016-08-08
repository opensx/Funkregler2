/*
 * AddrSelection.cpp
 *
 *  Created on: 04.08.2016
 *      Author: mblank
 *
 *      select a new loco address with the encoder
 */

#include "AddrSelection.h"

extern RotaryEncoderMax encoder;
extern Display7 disp;
extern FunkrEEPROM eep;

extern void userInteraction();

AddrSelection::AddrSelection() {

}

uint16_t AddrSelection::initFromEEPROM() {

	// read address mode
	_mode = eep.readAddrMode();

	if ((_mode != ADDR_MODE_SINGLE) && (_mode != ADDR_MODE_MULTI)) {
		_mode = ADDR_MODE_SINGLE;
		eep.writeAddrMode(_mode); // initialize new eeprom
	}
	_currentAddress = eep.readLastLoco();

	if (_currentAddress >= 100) {  // no valid SX address
		_currentAddress = 3;
		eep.writeLastLoco(_currentAddress);
	}
	_locoList = eep.readLocoList();
	if (_locoList.length() == 0) { // init
		// init
		_locoList = "locos=3,10,44";
		eep.writeLocoList(_locoList);
	}

	return _currentAddress;  // return always a valid address
}

void AddrSelection::start(uint16_t addr) {

	_mode = ADDR_MODE_SINGLE;
	_currentAddress = addr;
	_lastActiveAddress = addr;

	// read address mode
	_mode = eep.readAddrMode();

	if (_mode == ADDR_MODE_SINGLE) {
		_addresses[0] = addr;
		_selectedIndex = 0;
		_nAddresses = 1;
	} else {

		// check if the last loco is in new new locoList
		_selectedIndex = 0;  // there is always at least one loco
		for (int i = 0; i < _nAddresses; i++) {
			if (_addresses[i] == addr) {
				_selectedIndex = i;
				break;
			}
		}
		if (_currentAddress != _addresses[_selectedIndex]) { // loco has changed, update EEPROM
			// TODO should not be called here ------------ loco.setAddress(_addresses[_selectedIndex]);
			eep.writeLastLoco(_currentAddress);
		}
	}

	// init encoder to either use index table or the real address value
	if (_mode == ADDR_MODE_SINGLE) {
		encoder.setPosition(_currentAddress);
		disp.dispBlinkNumber(_currentAddress); // take address from address list
	} else {
		String locoList = eep.readLocoList();
		if (locoList.length() == 0) { // init
			// init
			locoList = "locos=3,10,44";
			eep.writeLocoList(locoList);
		}
		_nAddresses = parseLocoList(locoList);
		encoder.setPosition(_selectedIndex); // index of currently selected address
		disp.dispBlinkNumber(_addresses[_selectedIndex]); // take address from address list
	}
}

String AddrSelection::getLocos(void) {
	String s = eep.readLocoList();
	return s;
}

void AddrSelection::doLoop(void) {
	if (_mode == ADDR_MODE_SINGLE) {

		int newAddr = encoder.getPositionMax(MAX_LOCO_ADDRESS + 1);
		if (newAddr > MAX_LOCO_ADDRESS) {  // rollover
			encoder.setPosition(1);
			newAddr = 1;
		} else if (newAddr <= 0) {  // rollover
			encoder.setPosition(MAX_LOCO_ADDRESS);
			newAddr = MAX_LOCO_ADDRESS;
		}
		if (newAddr != _currentAddress) {
			_currentAddress = newAddr;
			disp.dispBlinkNumber(_currentAddress);
			userInteraction();  // resets switch off timer
		}
	} else { // list of selectable addresses is restricted
		int newIndex = encoder.getPositionMax(_nAddresses);
		if (newIndex < 0) {  // rollover
			encoder.setPosition(_nAddresses - 1);
			newIndex = _nAddresses - 1;
		} else if (newIndex >= _nAddresses) {
			encoder.setPosition(0);
			newIndex = 0;
		}
		if (newIndex != _selectedIndex) {
			/*	#ifdef _DEBUG
			 Serial.print("newIndex=");
			 Serial.println(newIndex);
			 #endif */
			_selectedIndex = (uint8_t) newIndex;
			// take new address from address list
			_currentAddress = _addresses[_selectedIndex];
			disp.dispBlinkNumber(_currentAddress);
			userInteraction();  // resets switch off timer
		}
	}
}

uint16_t AddrSelection::getMode(void) {
	return _mode;
}

bool AddrSelection::setMode(uint16_t mode) {
	if ((_mode == ADDR_MODE_SINGLE) || (_mode == ADDR_MODE_MULTI)) {
		_mode = mode;
		eep.writeAddrMode(_mode);
		return true;  // success
	} else {
		return false;
	}
}

bool AddrSelection::setModeString(String s) {
	s.toLowerCase();
	s.trim();
	if (s.startsWith("single")) {
		return setMode(ADDR_MODE_SINGLE);
	} else if (s.startsWith("multi")) {
		return setMode(ADDR_MODE_MULTI);
	} else {
		return false;
	}
}

String AddrSelection::getModeString(void) {
	if (_mode == ADDR_MODE_SINGLE) {
		return "single";
	} else if (_mode == ADDR_MODE_MULTI) {
		return "multi";
	} else {
		return "unknown";
	}

}

uint16_t AddrSelection::end(void) {

	eep.writeLastLoco(_currentAddress);
	return _currentAddress;
}

/** extract int[] of addresses from string
 *
 */
uint16_t AddrSelection::parseLocoList(String str) {

#ifdef _DEBUG
	Serial.print("parsing locos=");
	Serial.println(str);
#endif

	int index = 0;

	int pos = str.indexOf(",");
	while (pos != -1) {
		int a = str.toInt();  // first address
		if (index < 9) {
			_addresses[index] = a;
#ifdef _DEBUG
			Serial.print("adr=");
			Serial.println(a);
#endif
			index++;
		} else {
			break; // list too long
		}
		str = str.substring(pos + 1);  // start right after ',' char
		pos = str.indexOf(",");
		if (pos == -1) {
			//this is the last entry
			int a = str.toInt();  // first address
			_addresses[index] = a;
#ifdef _DEBUG
			Serial.print("adr=");
			Serial.println(a);
#endif
			index++;
		}
		_nAddresses = index;
	}
	if (index == 0) {
		// lets create at least one address
		_addresses[0] = 44;
		_nAddresses = 1;
	}
#ifdef _DEBUG
	Serial.print("_nAddresses=");
	Serial.println(_nAddresses);
#endif
	return _nAddresses;  // return the number of addresses we could parse

}

uint16_t AddrSelection::updateCurrentLocoFromLocoList(String locolist,
		uint16_t addr) {
	if (_mode == ADDR_MODE_SINGLE) {
		if (addr != _lastActiveAddress) {
			_currentAddress = addr;
			_lastActiveAddress = addr;
			eep.writeLastLoco(addr);
		}
		return addr;
	} else {

		parseLocoList(locolist);

		for (int i = 0; i < _nAddresses; i++) {
			if (_addresses[i] == addr) {
				// the current loco is in the list
				_selectedIndex = i;   // can have changed anyway !
				return addr;
			}
		}

		// if mode == MULTI and current loco not in list, reinit addr
		// select the first loco as default
		_selectedIndex = 0;
		_lastActiveAddress = _addresses[0];
		eep.writeLastLoco(_lastActiveAddress);
		return _lastActiveAddress;
	}
}
