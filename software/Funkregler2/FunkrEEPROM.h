/*
 * FunkrEEPROM.h
 *
 *  Created on: 04.08.2016
 *      Author: mblank
 */



#ifndef FUNKREEPROM_H_
#define FUNKREEPROM_H_

#include <Arduino.h>

#include "pcb-type.h"
#include <Eeprom24C32_64.h>
#include <Wire.h>


//******* EEPROM DEFINES ***********************************************
#define MAX_STRING_LEN  30       // reason: Wire library, max i2c message

#define EEPROM_SSID       100     // EEPROM addresses, non-overlapping ...
                                  // strings of length 30 are stored here
#define EEPROM_PASS       140
#define EEPROM_LOCOLIST   180

#define EEPROM_CCMODE     220    // 2 bytes
#define EEPROM_LAST_LOCO  230    // 2 bytes, address of last selected loco
#define EEPROM_ADDR_MODE  240    // 2 bytes Mode = all or = single


class FunkrEEPROM {
public:
	FunkrEEPROM ();

	void init(void);

    bool writeSSID(String);
    String readSSID(void);

    bool writePASS(String);
    String readPASS(void);

    bool writeLocoList(String);
    String readLocoList(void);

    bool writeLastLoco(uint16_t);
    uint16_t readLastLoco(void);

    bool writeAddrMode(uint16_t);
    uint16_t readAddrMode(void);


private:

    bool writeConfigString(word addr, String);
    String readConfigString(word addr);
    
    bool writeConfigUInt16(word addr, uint16_t);
    uint16_t readConfigUInt16(word addr);
 

};



#endif /* FUNKREEPROM_H_ */
