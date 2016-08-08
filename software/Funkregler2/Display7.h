/*
 * Display7.h
 *
 * for a 2 digit 7-segment LED display (with mkr1000)
 * WiFi-Fredi 0.2 (with mkr1000)
 *
 *  Created on: 20.07.2016
 *      Author: mblank
 */

#ifndef DISPLAY7_H_
#define DISPLAY7_H_

#include <Arduino.h>
#include "pcb-type.h"



#define NCHAR   17
#define NSEG     8      // a   b   c  d   e  f  g  DP
#define DP_SEG   7
#define D_ON     1
#define D_OFF    0
#define BLINK    2      // set 2 for blinking (0=off, 1=on)


class Display7 {

public:
	Display7(void);
	void init(void);
	void switchOn(int digit);  // switch on left or right display
	void setDecPoint(int disp, bool on);  // switch on/off decimal point left or right
	void decBlink(int disp);   // set the decimal point to blink
	void doDisplay(int digit);
	void switchOff();
	void dispChar(int d, char c);
	void blinkChar(int d, char c);
	void dispCharacters(char c1, char c2);
	void blinkCharacters(char c1, char c2);
	void dispError(char c2);   // 'Ec2' blinking
	void dispNumber(int n);
  void dispNumberSigned(int n, bool back);
	void dispBlinkNumber(int n);
	void setDim(bool);
	bool isDimming();
	void setFlicker(bool);
	void test(void);

private:

    bool _flickerOn;
    volatile int _segmentOn[2][NSEG];  // [right/left][ a b .... DP]
	 bool _dim;

    //char ch[NCHAR];
    //int number[NCHAR][7];
};



#endif /* DISPLAY7_H_ */

