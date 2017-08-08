/* SXLoco.h
 * definitions for an Selectrix Loco
 *
 */
#ifndef DCCLOCO_H_
#define DCCLOCO_H_

#include "dccutils.h"

class DCCLoco {
public:
    DCCLoco();
    DCCLoco(int16_t address);

    uint8_t toggleFunction(uint8_t i);
    
    int16_t setSpeed(int16_t);  // speed (signed), -127 .. + 127
    int16_t updateSpeed(int16_t); // arbitrary value, from encoder
    int16_t getSpeed(void);  // speed (signed)
    uint16_t getAbsSpeed(void);  // speed (positive)
    
    uint8_t getBackward(void);
    void setBackward(bool);
    void stop(void);    // set speed to zero, doesn't change dir
    

    uint16_t getAddress(void);
    uint16_t setAddress(uint16_t);

    uint8_t hasChanged(void); 
    void resetChanged(void);

    uint8_t getDCCData(void);
    void setFromDCCData(uint8_t);

    String getF0F4String(void);
    String getDCCppFunctionsString(void);  // function string for DCCpp

    void init(void);

private:
    
    // address gets calculated from addresses[] table
    uint8_t  _dir;  // contains direction
    uint8_t  _changed;  // loco address has changed
    uint16_t _address;  // the address of the current loco
    uint16_t _absSpeed;   // abs(speed), no sign
    uint8_t  _functions;   // contains state of functions F0 ... F4
    String   _functionsString;  // contains state of functions F0 ... F4
    String   _functionsStringDCCpp;
    uint32_t _modTime;  // last time the speed was updated from rotary encoder

    void _setDCCppFunctionsString(void);  // calculate the DCCpp functions String
};

#endif // DCCLOCO_H_


