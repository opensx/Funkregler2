#Software für den Arduino des Funkregler2, HW siehe:

<a href="http://opensx.net/funkregler2" _target="blank"> Funkregler2 auf opensx </a>

#Funkregler2.ino

version for MKR1000 and 2 (or 4) digit 7-segment display

BETA Version 

Hardware rev. 0.3 and rev. DCC-0.1

- Rotary Encoder für Speed und Adress Selection
- Buttons für Adress-Selection ("A"), Licht (F0="L") und Function (F1="F")
- DCC-Version: Buttons for A and F0 ... F4

15 Sep 2016 - improved config  
13 Aug 2016 - for DCC HW 
08 Aug 2016 - initial Version

#Libraries needed
- SPI
- WiFi101
- WiFiUdp
- Wire
- Eeprom24C32_64
- Timer5  (timer for SAMD)
- Adafruit_SleepyDog
- AnalogButtons (for DCC HW with 6 Buttons)

You can use the "library manager" of the arduino IDE to install the 
libraries, see
<a href="https://www.arduino.cc/en/Guide/Libraries" target="_blank">Arduino 
Libraries Guide</a>

 (c) Michael Blank - 2016 - <a href="http://opensx.net" target="_blank">
opensx.net</a>
