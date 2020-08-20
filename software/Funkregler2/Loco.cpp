/* Loco.cpp
 * contains all details of a selectrix(SX1) loco
 
 * cannot include "EEPROM.h" here:
 * "This seems to be a general problem of the Arduino IDE: 
 * It only recognizes libraries that are included in the 
 * (primary) .ino file.
 * solution: include EEPROM.h only in .ino file "
 */

#include "utils.h"
#include "Loco.h"

Loco::Loco()
{
	_address = 3;
	_sxData = 0; // contains Speed, dir, F0, F1
	_changed = 1;
	_speed = 0; // signed speed used for DCC only
}

Loco::Loco(int16_t a)
{
	uint8_t amax = 103;
	if (ccmode != CCMODE_SX)
		amax = 99;
	if ((a > 0) && (a <= amax))
	{
		_address = (uint8_t)a;
	}
	else
	{
		_address = 3;
	}
	_sxData = 0;
	_changed = 1;
	_speed = 0;
}

uint8_t Loco::hasChanged()
{
	return _changed;
}

void Loco::resetChanged()
{
	_changed = 0;
}

void Loco::stop()
{
	_sxData &= 0xE0; // set speed bits (only) to zero
	_speed = 0;
	_changed = 1;
}

int8_t Loco::getSpeed()
{
	if (ccmode == CCMODE_SX)
	{
		int16_t s = _sxData & 0x1F; // positive
		if (bitRead(_sxData, 5))
		{
			return -s;
		}
		else
		{
			return s;
		}
	}
	else
	{
		if (bitRead(_sxData, 5))
		{
			return -_speed;
		}
		else
		{
			return _speed;
		}
	}
}

uint8_t Loco::toggleF0()
{
	if (bitRead(_sxData, 6))
	{
		bitClear(_sxData, 6);
	}
	else
	{
		bitSet(_sxData, 6);
	}
	_changed = 1;
	return bitRead(_sxData, 6);
}

uint8_t Loco::getF0()
{
	return bitRead(_sxData, 6);
}

uint8_t Loco::toggleF1()
{
	if (bitRead(_sxData, 7))
	{
		bitClear(_sxData, 7);
	}
	else
	{
		bitSet(_sxData, 7);
	}
	_changed = 1;
	return bitRead(_sxData, 7);
}

uint8_t Loco::toggleFunction(uint8_t i)
{
	if (i == 0)
	{
		return toggleF0();
	}
	else if (i == 1)
	{
		return toggleF1();
	}
	else
	{
		return 0; // there are no functions F2, F3, F4 with Selectrix
				  // TODO implement for DCC/LN
	}
}

uint8_t Loco::getF1()
{
	return bitRead(_sxData, 7);
}

uint8_t Loco::getBackward()
{
	return bitRead(_sxData, 5);
}

void Loco::setBackward(bool b)
{
	if (b == true)
	{
		bitSet(_sxData, 5);
	}
	else
	{
		bitClear(_sxData, 5);
	}
}

uint16_t Loco::getAddress()
{
	return _address;
}

uint16_t Loco::setAddress(uint16_t a)
{
	uint8_t amax = 103;
	if (ccmode != CCMODE_SX)
		amax = 99;
	if ((a > 0) && (a <= amax))
	{
		_address = (uint8_t)a;
	}
	else
	{
		_address = 3;
	}
	return _address;
}

uint8_t Loco::getSXData()
{
	// sx data (bit 5=direction, bit 6= light, bit7= horn)
	return _sxData;
}

void Loco::setFromSXData(uint8_t data)
{
	// sx data (bit 5=direction, bit 6= light, bit7= horn)
	_sxData = data;
}

// maximum value of speed
int8_t Loco::getMaxSpeed()
{
	if (ccmode == CCMODE_SX)
	{
		return 31;
	}
	else
	{
		return 64; // will be mult. by 2
	}
}

/** set the speed from number -31 .. +31
 *  
 */
void Loco::setSpeed(int8_t sp)
{
	//Serial.print("SXLoco.setSpeed, sp=");
	//Serial.println(sp);

	if (sp > getMaxSpeed())
		sp = getMaxSpeed();
	if (sp < -getMaxSpeed())
		sp = -getMaxSpeed();

	uint8_t _newSpeed;
	uint8_t _newBackward = 0;

	// do not change direction when new speed == 0
	if (sp < 0)
	{
		_newBackward = 1;
	}
	else if (sp > 0)
	{
		_newBackward = 0;
	}
	_newSpeed = abs(sp);

	if (ccmode == CCMODE_SX)
	{
		// did speed or direction change ?
		if ((_newSpeed != (_sxData & 0x1F)) || (_newBackward != bitRead(_sxData, 5)))
		{
			_sxData = (_sxData & 0xE0) | _newSpeed;
			if (_newBackward)
			{
				bitSet(_sxData, 5);
			}
			else
			{
				bitClear(_sxData, 5);
			}
			_changed = 1;
		}
	}
	else
	{
		// did speed or direction change ?
		if ((_newSpeed != _speed) || (_newBackward != bitRead(_sxData, 5)))
		{
			_speed = _newSpeed;
			if (_newBackward)
			{
				bitSet(_sxData, 5);
			}
			else
			{
				bitClear(_sxData, 5);
			}
			_changed = 1;
		}
	}
}
