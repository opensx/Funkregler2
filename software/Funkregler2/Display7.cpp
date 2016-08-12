/*
 * Display7.cpp
 *
 *  for a 2 digit 7-segment LED display
 *  WiFi-Fredi 0.2 (with mkr1000)
 *
 *  Created on: 20.07.2016
 *      Author: mblank
 */

#include "Display7.h"

// all characters we can display
static char _ch[NCHAR] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A',
		'C','c','E','F', 'o','-',' ' };

// segments to switch on for a character (segment 1= bottom)
static int _number[NCHAR][7] = { { 1, 1, 1, 1, 1, 1, 0 },  // 0
		{ 0, 0, 0, 0, 1, 1, 0 },  // 1
		{ 1, 1, 0, 1, 1, 0, 1 },  // 2
		{ 1, 0, 0, 1, 1, 1, 1 },  // 3
		{ 0, 0, 1, 0, 1, 1, 1 },  // 4
		{ 1, 0, 1, 1, 0, 1, 1 },  // 5
		{ 1, 1, 1, 1, 0, 1, 1 },  // 6
		{ 0, 0, 0, 1, 1, 1, 0 },  // 7
		{ 1, 1, 1, 1, 1, 1, 1 },  // 8
		{ 1, 0, 1, 1, 1, 1, 1 },  // 9
		{ 0, 1, 1, 1, 1, 1, 1 },  // A
		{ 1, 1, 1, 1, 0, 0, 0 },  // C
		{ 1, 1, 0, 0, 0, 0, 1 },  // c
		{ 1, 1, 1, 1, 0, 0, 1 },  // E
		{ 0, 1, 1, 1, 0, 0, 1 },  // F
		{ 1, 1, 0, 0, 0, 1, 1 },  // o
		{ 0, 0, 0, 0, 0, 0, 1 },  // -
    { 0, 0, 0, 0, 0, 0, 0 }   // blank
};

// pins on arduino to drive segments, active low
#ifdef HWREV_0_2A
static int _segments[NSEG] = { 4, 13, A2, A3, 10, 2, A4, 1 };
#elif defined(HWREV_0_2)
static int _segments[NSEG] = { 4, 13, 12, 11, 10, 2, A4, 1 };
#elif defined(HWREV_0_3)
static int _segments[NSEG] = { 4, 13, A2, A3, 10, 2, A4, 1 };
#elif defined(HWREV_D_0_1)
static int _segments[NSEG] = { 4, 13, A2, 3, 10, 2, A6, 1 };
#endif

Display7::Display7(void) {

}

void Display7::test(void) {
	pinMode(ANODE1, OUTPUT);
	digitalWrite(ANODE1, LOW);  // off

	for (int i = 0; i < NSEG; i++) {
		pinMode(_segments[i], OUTPUT);
		digitalWrite(_segments[i], LOW);
		delay(1000);
	}

	digitalWrite(ANODE1, HIGH);  // off
	digitalWrite(ANODE2, LOW);
	delay(1000);
	digitalWrite(ANODE2, HIGH);  // off

}

void Display7::init(void) {
	pinMode(ANODE1, OUTPUT);
	pinMode(ANODE2, OUTPUT);
	digitalWrite(ANODE1, HIGH);  // off
	digitalWrite(ANODE2, HIGH);  // off
#ifdef DIGITS4
  pinMode(ANODE3, OUTPUT);
  pinMode(ANODE4, OUTPUT);
  digitalWrite(ANODE3, HIGH);  // off
  digitalWrite(ANODE4, HIGH);  // off
#endif

	for (int i = 0; i < 8; i++) {
		pinMode(_segments[i], OUTPUT);
	}

	for (int i = 0; i < 8; i++) {
		digitalWrite(_segments[i], LOW);
	}
	_flickerOn = false;
	_dim = false;

}

// never switch on both!
// (SAMD21 max io current = 7mA)
void Display7::switchOn(int digit) {

	digitalWrite(ANODE1, HIGH);
	digitalWrite(ANODE2, HIGH);
#ifdef DIGITS4
  digitalWrite(ANODE3, HIGH);
  digitalWrite(ANODE4, HIGH);
#endif

	if (digit == 0) {
		digitalWrite(ANODE1, LOW);
	} else if (digit == 1) {
		digitalWrite(ANODE2, LOW);
	} else if (digit == 2) {
#ifdef DIGITS4
    digitalWrite(ANODE3, LOW);
#endif
  } else if (digit == 3) {
#ifdef DIGITS4
    digitalWrite(ANODE4, LOW);
#endif
  }
  // else switch both off
}

void Display7::setDecPoint(int disp, bool state) {
	if (state == true) {
		_segmentOn[disp][DP_SEG] = 1;
	} else {
		_segmentOn[disp][DP_SEG] = 0;
	}
}

void Display7::decBlink(int disp) {
	_segmentOn[disp][DP_SEG] = BLINK;
}

void Display7::doDisplay(int d) {
	static long blinkTimer = 0;

	int blinkState =0;
	if ((millis() - blinkTimer) <= 500 ) {
		blinkState = 0;
	} else if ((millis() - blinkTimer) <= 1000) {
		blinkState = 1;
	} else {
		// reset timer
		blinkTimer = millis();
	}
	switchOn(d);
	for (int i = 0; i < NSEG; i++) {
		switch (_segmentOn[d][i]) {
			case 1:
			  digitalWrite(_segments[i], LOW);  // ON
			  break;
		   case 0:
			  digitalWrite(_segments[i], HIGH);  // OFF
			  break;
			case 2:
			  digitalWrite(_segments[i], blinkState);	 // timer dependent
		}
	}

}

void Display7::switchOff() {
	digitalWrite(ANODE1, HIGH);
	digitalWrite(ANODE2, HIGH);
#ifdef DIGITS4
  digitalWrite(ANODE3, HIGH);
  digitalWrite(ANODE4, HIGH);
#endif
}


void Display7::dispChar(int d, char c) {

	for (int i = 0; i < NCHAR; i++) {
		if (c == _ch[i]) {
			for (int j = 0; j < 7; j++) {
				if (_number[i][j] == 0) {
					_segmentOn[d][j] = 0;
				} else {
					_segmentOn[d][j] = 1;
				}
			}
		}
	}
}

void Display7::blinkChar(int d, char c) {

	for (int i = 0; i < NCHAR; i++) {
		if (c == _ch[i]) {
			for (int j = 0; j < 7; j++) {
				if (_number[i][j] == 0) {
					_segmentOn[d][j] = 0;
				} else {
					_segmentOn[d][j] = 2;
				}
			}
			break;
		}
	}
}

void Display7::dispCharacters(char c1, char c2) {
#ifdef DIGITS4
  dispChar(0,' ');
  dispChar(3,' ');
  dispChar(2, c1);
  dispChar(1, c2);
#else
	dispChar(1, c1);
	dispChar(0, c2);
#endif
}

void Display7::dispCharacters(char c1, char c2, char c3, char c4) {
  dispChar(1, c1);
  dispChar(0, c2);
  dispChar(2, c3);
  dispChar(3, c4);
  
}

void Display7::blinkCharacters(char c1, char c2) {
#ifdef DIGITS4
  blinkChar(2, c1);
  blinkChar(1, c2);
  blinkChar(0,' ');
  blinkChar(3,' ');
#else
	blinkChar(1, c1);
	blinkChar(0, c2);
#endif
}

void Display7::blinkCharacters(char c1, char c2, char c3, char c4) {
  blinkChar(1, c1);
  blinkChar(0, c2);
  blinkChar(2, c3);
  blinkChar(3, c4);
}

void Display7::dispError(char c2) {
#ifdef DIGITS4
  blinkChar(2, 'E');
  blinkChar(1, c2);
  blinkChar(0,' ');
  blinkChar(3,' ');
#else
  blinkChar(1, 'E');
	blinkChar(0, c2);
#endif
}

void Display7::dispNumber(int n) {
  setDecPoint(BW,false);
  setDecPoint(FW,false);
#ifdef DIGITS4
  if ((n >= 10000) || (n < 0)) {
    // error
    dispChar(2, '-');
    dispChar(1, '-');
  } else {
    int z = n / 10;
    dispChar(2, '0' + z);
    dispChar(1, '0' + (n - 10 * z));
  }
  dispChar(0,' ');
  dispChar(3,' ');
#else
	if ((n >= 100) || (n < 0)) {
		// error
		dispChar(1, '-');
		dispChar(0, '-');
	} else {
		int z = n / 10;
		dispChar(1, '0' + z);
		dispChar(0, '0' + (n - 10 * z));
	}
#endif
}

void Display7::dispNumberSigned(int nsigned, bool backward) {
  dispNumber(abs(nsigned)); // decimal points are blanked here
  // now set the FW or BW LED
  if (backward) {
     setDecPoint(BW,true);
     setDecPoint(FW,false);
  } else {
     setDecPoint(BW,false);
     setDecPoint(FW,true);
  }
}
   
void Display7::dispBlinkNumber(int n) {
	setDecPoint(BW,false);
	setDecPoint(FW,false);
#ifdef DIGITS4
  if ((n >= 10000) || (n < 0)) {
    // error
    blinkChar(2, '-');
    blinkChar(1, '-');
  } else {
    int z = n / 10;
    blinkChar(2, '0' + z);
    blinkChar(1, '0' + (n - 10 * z));
    dispChar(0,' ');
    dispChar(3,' ');
  }
#else
	if ((n >= 100) || (n < 0)) {
		// error
		blinkChar(1, '-');
		blinkChar(0, '-');
	} else {
		int z = n / 10;
		blinkChar(1, '0' + z);
		blinkChar(0, '0' + (n - 10 * z));
	}
#endif
}


void Display7::setFlicker(bool fliOn) {
	_flickerOn = fliOn;
}

void Display7::setDim(bool di) {
	_dim = di;
}

bool Display7::isDimming() {
	return _dim;
}


