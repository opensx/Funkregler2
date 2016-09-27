/*
 * AddrSelection.h
 *
 *  Created on: 04.08.2016
 *      Author: mblank
 */

#ifndef _ADDRSELECTION_H_
#define _ADDRSELECTION_H_

#include <Arduino.h>
#include "pcb-type.h"
#include "RotaryEncoderMax.h"
#include "Display7.h"
#include "FunkrEEPROM.h"

class AddrSelection {
public:
	AddrSelection();

	uint16_t initFromEEPROM();  // init from eeprom and return last selected loco

    void doLoop(boolean single);

    void start(uint16_t addr, boolean single);  // start to select a new address
    uint16_t end(void);  // returning new address

    String getLocos(void);  // loco list string
    uint16_t addLocoToLocoList(uint16_t addr); // add new selected loco to locolist


private:


    uint16_t _lastActiveAddress;   // last used address
    uint16_t _currentAddress;      // during address selection active

    String _locoList;

    uint16_t _addresses[10] = { 1 };  // contains all valid loco addresses
    uint16_t _nAddresses; // number of valid loco addresses { 0, 1, ... (nAddresses-1) }
    uint16_t _selectedIndex = 0; // currently selected loco index (a = addresses[index] )

    uint16_t parseLocoList(String str);   // extract int[] of addresses from string

};



#endif /* _ADDRSELECTION_H_ */
