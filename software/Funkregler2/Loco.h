/* Loco.h
 * definitions for an Selectrix Loco
 *
 */
#ifndef LOCO_H_
#define LOCO_H_

#include "utils.h"

class Loco
{
public:
    Loco();
    Loco(int16_t address);

    uint8_t toggleF0(void);
    uint8_t getF0(void);

    uint8_t toggleF1(void);
    uint8_t getF1(void);

    uint8_t toggleFunction(uint8_t i);

    void setSpeed(int16_t); // speed (signed)
    int16_t getSpeed(void); // speed (signed)

    uint8_t getBackward(void);
    void setBackward(bool);
    void stop(void); // set speed to zero, doesn't change dir

    uint16_t getAddress(void);
    uint16_t setAddress(uint16_t);

    uint8_t hasChanged(void);
    void resetChanged(void);

    uint8_t getSXData(void);
    void setFromSXData(uint8_t);

private:
    // address gets calculated from addresses[] table
    uint8_t _sxData = 0;   // contains ALL (current) loco data
    uint8_t _changed = 1;  // loco address has changed
    uint16_t _address = 1; // the address of the current loco
    int16_t _speed = 0;    // for dcc
};

#endif // LOCO_H_
