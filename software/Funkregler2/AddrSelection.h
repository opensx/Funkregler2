/*
 * AddrSelection.h
 *
 *  Created on: 04.08.2016
 *      Author: mblank
 */

#ifndef LIBRARIES_EEPROM24C32_64_ADDRSELECTION_H_
#define LIBRARIES_EEPROM24C32_64_ADDRSELECTION_H_

#include <Arduino.h>
#include "pcb-type.h"
#include "RotaryEncoderMax.h"
#include "Display7.h"
#include "FunkrEEPROM.h"

#define ADDR_MODE_SINGLE    0       // use only one address (but this is selectable)
#define ADDR_MODE_MULTI     1       // use a fixed list of loco addresses

class AddrSelection {
public:
	AddrSelection();

	uint16_t initFromEEPROM();  // init from eeprom and return last selected loco

    void doLoop(void);

    void start(uint16_t addr);  // start to select a new address
    uint16_t end(void);  // returning new address

    String getModeString(void);
    bool setModeString(String);  // set to "single" or "multi"

    String getLocos(void);  // loco list string
    uint16_t updateCurrentLocoFromLocoList(String, uint16_t); // check if current loco is still valid after locolist update


private:

    bool setMode(uint16_t mode);
    uint16_t getMode(void);

    uint16_t _lastActiveAddress;   // last used address
    uint16_t _currentAddress;      // during address selection active
    uint16_t _mode;                // single or multi address mode (see above)

    String _locoList;

    uint16_t _addresses[10] = { 1 };  // contains all valid loco addresses
    uint16_t _nAddresses; // number of valid loco addresses { 0, 1, ... (nAddresses-1) }
    uint16_t _selectedIndex = 0; // currently selected loco index (a = addresses[index] )

    uint16_t parseLocoList(String str);   // extract int[] of addresses from string

};



#endif /* LIBRARIES_EEPROM24C32_64_ADDRSELECTION_H_ */
