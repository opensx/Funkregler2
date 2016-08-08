/** sxutils.cpp
 * origianall contained some optimized string functions
 * these PAYLOAD functions are not needed for the SAMD21
 * which is used in xbee data transfer (API mode)
 * string always ends with a newline char
 * and is always zero terminated
 */

#include "sxutils.h"
#include "stdio.h"

uint8_t lastAddr[N_ADDR]={64,78,1,3};  // for storing the last 4 addresses  - TODO read from EEPROM
uint8_t addrUsage[N_ADDR];  // store usage of last 4 addresses
uint8_t indexLastAddr=1;

/**
add the 4 last address as indirect_address = 100 ...103 to the
number of possible addresses.
for 100, return lastAddr[0] and so on
*/
uint8_t addressFunction(uint8_t aIn) {
    if ((aIn > MAX_ADDRESS) &&
        (aIn <= MAX_ADDRESS+N_ADDR) ) {
        return lastAddr[aIn-MAX_ADDRESS-1];
    } else {
        return aIn;
    }
}



