/*
 * FunkrEEPROM.cpp
 *
 *  Created on: 04.08.2016
 *      Author: mblank
 */

#include "FunkrEEPROM.h"

//************** i2c eeprom ******************************************
#define EEPROM_I2C_ADDR    0x50
Eeprom24C32_64 eeprom(EEPROM_I2C_ADDR);  // EEPROM at i2c address 50hex

FunkrEEPROM::FunkrEEPROM() {

}

// Initiliaze EEPROM library.
void FunkrEEPROM::init(void) {
	eeprom.initialize();    // = Wire.begin();
	delay(10);
}

bool FunkrEEPROM::writeSSID(String s) {
	return writeConfigString((word) EEPROM_SSID, s);
}

String FunkrEEPROM::readSSID(void) {
	return readConfigString((word) EEPROM_SSID);
}

bool FunkrEEPROM::writePASS(String s) {
	return writeConfigString((word) EEPROM_PASS, s);
}

String FunkrEEPROM::readPASS(void) {
	return readConfigString((word) EEPROM_PASS);
}


bool FunkrEEPROM::writeLocoList(String s) {
	return writeConfigString((word) EEPROM_LOCOLIST, s);
}


String FunkrEEPROM::readLocoList(void) {
	return readConfigString((word) EEPROM_LOCOLIST);
}


bool FunkrEEPROM::writeAddrMode(uint16_t m) {
	return writeConfigUInt16((word) EEPROM_ADDR_MODE, m) ;
}


uint16_t FunkrEEPROM::readAddrMode(void) {
	return readConfigUInt16((word) EEPROM_ADDR_MODE) ;
}

bool FunkrEEPROM::writeLastLoco(uint16_t a) {
	return writeConfigUInt16((word) EEPROM_LAST_LOCO, a) ;
}

uint16_t FunkrEEPROM::readLastLoco(void) {
	return readConfigUInt16((word) EEPROM_LAST_LOCO) ;
}


/**
 * write a (non-empty) config string to EEPROM
 */
bool FunkrEEPROM::writeConfigString(word addr, String s) {
	char eebuf[MAX_STRING_LEN] = { 0 };
	if (s.length() == 0) return false;

	s.toCharArray(eebuf, MAX_STRING_LEN);
	eeprom.writeBytes((word) addr, (word) MAX_STRING_LEN,
					(byte *) eebuf);  // write '0' at the end of the string also
	delay(20);
	return true;
}

/** read a config string from EEPROM
 *
 */
String FunkrEEPROM::readConfigString(word addr) {
	char buffer[MAX_STRING_LEN + 1] = { 0 };
	eeprom.readBytes((word) addr, (word) MAX_STRING_LEN,
			(byte *) buffer);
	if (buffer[0] == 0xff) {
		return String();   // empty, not initialized
	} else {
		return String(buffer);
	}
}

uint16_t FunkrEEPROM::readConfigUInt16(word addr) {
	uint16_t d = 0;

	byte low = eeprom.readByte(addr);
	byte high = eeprom.readByte(addr+1);

	d = (((uint16_t)high ) << 8) | low;
	return d;

}

bool FunkrEEPROM::writeConfigUInt16(word addr, uint16_t d) {

	byte low = d & 0xff;
	byte high = (d >> 8) && 0xff;

	eeprom.writeByte(addr,low);
	eeprom.writeByte(addr+1, high);

	delay(20);
	return true;
}
