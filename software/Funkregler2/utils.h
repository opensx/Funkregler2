/** utils.h
 * for filling the payload string with information
 *
 * (c) Michael Blank
 *
 * 03.10.2015 initial version
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <Arduino.h>
#include <inttypes.h>

#define MIN_ADDRESS 1
#define MAX_ADDRESS 99
#define INVALID_UI8 255 // invalid uint8_t value
#define INVALID (-1)

// TrackPower (Gleisspannungsbit) states
#define POWER_UNKNOWN 2
#define POWER_OFF 0
#define POWER_ON 1

#define EEPROM_START 10 // store relative index (0..3) into current address
// last used loco
#define N_ADDR 4 // store the last 4 addresses

extern uint8_t lastAddr[N_ADDR];  // for storing the last 4 addresses
extern uint8_t addrUsage[N_ADDR]; // store usage of last 4 addresses
extern uint8_t indexLastAddr;

#define N_CCMODE (2)
#define CCMODE_SX (0)
#define CCMODE_LN (1)

extern String s_ccmode[N_CCMODE];
extern uint8_t ccmode;

extern uint8_t addressFunction(uint8_t addr);

#endif //UTILS_H_
