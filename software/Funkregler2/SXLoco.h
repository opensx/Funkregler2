/* SXLoco.h
 * definitions for an Selectrix Loco
 *
 */
#ifndef SXLOCO_H_
#define SXLOCO_H_

#include "sxutils.h"

class SXLoco {
public:
	SXLoco();
	SXLoco(int);

	uint8_t toggleF0(void);
    uint8_t getF0(void);
    
    uint8_t toggleF1(void);
    uint8_t getF1(void);

    uint8_t toggleFunction(uint8_t i);
    
    void setSpeed(int);  // speed (signed)
    int getSpeed(void);  // speed (signed)
    
    uint8_t getBackward(void);
    void setBackward(bool);
    void stop(void);    // set speed to zero, doesn't change dir
    

    int getAddress(void);
    int setAddress(int);

    uint8_t hasChanged(void); 
    void resetChanged(void);

    uint8_t getSXData(void);
    void setFromSXData(uint8_t);
    

private:
    
    // address gets calculated from addresses[] table
    uint8_t _sxData = 0;  // contains ALL (current) loco data
    uint8_t _changed = 1;  // loco address has changed
    uint8_t _address = 1;  // the address of the current loco
};

#endif // SXLOCO_H_


