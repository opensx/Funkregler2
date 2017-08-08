/* pcb-type.h
 * definitions for the PCBoard of Funkregler-WiFi
 *
 */
#ifndef PCB_TYPE_H_
#define PCB_TYPE_H_

#include <Arduino.h>
//********************* general - HW independent

//#define HWREV_0_3   // test pcb rev 0.3 with EEPROM
//#define HW_STRING "HW_0.3"

#define HWREV_D_0_1
#define HW_STRING "HW_D0.1"

#define MAX_LOCO_ADDRESS    99   // for selectrix


#define INVALID_INT  (10000)   // invalid integer value, higher than any loco addr or speed


//******* AD calibration **********************************************

#define VOLT_3300   3200     // default Battery-AD calibration

//******* HARDWARE PINS ********** REV 0.2(a) *************************
#ifdef HWREV_0_2A

#define ANODE1   A5
#define ANODE2   A0


#define ENC1    6   // first encoder pin
#define ENC2    8   // second encoder pin

#define DP      1

#define BATT_PIN   (A1)   // battery is at A1, connected by 2:1 divider
#define FW         0    // display-number for forward direction
#define BW         1    // display-number for backward direction
#define STOP_BTN  7     // encoder push button
// #define BATT_ON    5     // for switching batt power on  FALSCHER PIN = ADDR_BTN !!!

#define ADDR_BTN   5     // for address selection
#define F0_BTN     3      // toggle button
#define F1_BTN     9     // momentary function

#define IRQ_FREQ   200

//******* HARDWARE PINS ******** REV 0.3 **********************
#elif defined (HWREV_0_3)

#define ANODE1   A5          // left side 7-seg enable (active low)
#define ANODE2   A0          // right side 7-seg enable  (active low)

#define ENC1    6   // first encoder pin
#define ENC2    8   // second encoder pin

#define DP      1

#define BATT_PIN   (A1)    // battery is at A1, connected by 2:1 divider
#define FW         0    // display-number for forward direction
#define BW         1    // display-number for backward direction
#define STOP_BTN  7     // encoder push button
#define BATT_ON    0     // for switching batt power on / off

#define ADDR_BTN   5     // for address selection
#define F0_BTN     3      // toggle button
#define F1_BTN     9     // momentary function

#define IRQ_FREQ   200

//******* HARDWARE PINS ******** TODO !!!!  REV 0.4 **********
#elif defined (HWREV_0_4)

#define ANODE1   A5          // left side 7-seg enable (active low)
#define ANODE2   A0          // right side 7-seg enable  (active low)

#define LED_F0   9
#define LED_F1   14

#define ENC1    6   // first encoder pin
#define ENC2    8   // second encoder pin

#define DP      1

#define BATT_PIN   (A1)    // battery is at A1, connected by 2:1 divider
#define FW         0    // display-number for forward direction
#define BW         1    // display-number for backward direction
#define STOP_BTN  7     // encoder push button
#define BATT_ON    0     // PIN for switching batt power on / off

// #define ADDR_BTN   5     // for address selection
// #define F0_BTN     3      // toggle button
// #define F1_BTN     9     // momentary function

//************ 3 analog Buttons ****************************

#define ANALOG_BTN_PIN   A2

#define IRQ_FREQ   200

//******* HARDWARE PINS ******** DCC (4digit) 0.1 **************
#elif defined (HWREV_D_0_1)

#define DIGITS4
#define ANODE1   9          // 1st 7-seg enable (active low)
#define ANODE2   A4           // 2nd 7-seg enable  (active low)
#define ANODE3   A3          // 3rd 7-seg enable  (active low)
#define ANODE4   A0          // 4th 7-seg enable  (active low)

#define ENC1    6   // first encoder pin
#define ENC2    8   // second encoder pin

#define DP      1

#define BATT_PIN   A1    // battery is at A1, connected by 2:1 divider
#define FW         3    // display-number for forward direction
#define BW         0    // display-number for backward direction
#define STOP_BTN  7     // encoder push button
#define BATT_ON    0     // for switching batt power on / off


//************ 1 digital Buttons ***************************
#define ADDR_BTN   5     // for address selection (digital)

//************ 5 analog Buttons ****************************

#define ANALOG_BTN_PIN   A5
#define F0_BTN     3      // toggle button
#define F1_BTN     9     // momentary function
#define F2_BTN     3      // toggle button
#define F3_BTN     9     // momentary function
#define F4_BTN     3      // toggle button

#define IRQ_FREQ   400    // needs to be higher for 4digit display

#else   // ************************************************
#error Hardware Revision not supported
#endif // *************************************************



#endif //PCB_TYPE_H_
