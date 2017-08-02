/** sxutils.h
 * for filling the payload string with information
 *
 * (c) Michael Blank
 *
 * 03.10.2015 initial version
 */

#ifndef DCCUTILS_H_
#define DCCUTILS_H_

#include <Arduino.h>
#include <inttypes.h>

// SELECTRIX SPECIFIC
#define MAX_SPEED  127     // min_speed = 0 ! 
#define MIN_ADDRESS   1
#define MAX_ADDRESS   9999
#define INVALID_UI8   255  // invalid uint8_t value

// TrackPower (Gleisspannungsbit) states
#define POWER_UNKNOWN   2
#define POWER_OFF       0
#define POWER_ON        1


#define EEPROM_START   10   // store relative index (0..3) into current address
   // last used loco
#define N_ADDR         4    // store the last 4 addresses


extern uint16_t lastAddr[N_ADDR];  // for storing the last 4 addresses
extern uint8_t addrUsage[N_ADDR];  // store usage of last 4 addresses
extern uint8_t indexLastAddr;

uint16_t addressFunction(uint8_t addr);

#endif  //DCCUTILS_H_


