#Software für den Arduino des Funkregler2, HW siehe:

<a href="http://opensx.net/funkregler2" _target="blank"> Funkregler2 auf opensx </a>

#Funkregler2.ino

version for MKR1000 and 2 digit 7-segment display

BETA Version - don't use.

Hardware rev. 0.3

- Rotary Encoder für Speed und Adress Selection
- Buttons für Adress-Selection ("A"), Licht (F0="L") und Function (F1="F")

08 Aug 2016 - initial Version

#Libraries needed
- SPI
- WiFi101
- WiFiUdp
- Wire
- Eeprom24C32_64
- Timer5  (timer for SAMD)
- Adafruit_SleepyDog

You can use the "library manager" of the arduino IDE to install the 
libraries, see
<a href="https://www.arduino.cc/en/Guide/Libraries" target="_blank">Arduino 
Libraries Guide</a>

 (c) Michael Blank - 2016 - <a href="http://opensx.net" target="_blank">
opensx.net</a>
