/*****************************************************************
 Funkregler2DCC.ino

 version for MKR 1000 and 7-segment display  DCC !!!

 Rotary Encoder for Speed and Address Selection
 Buttons for Start-Address-Selection ("A"), Light (F0="L") and Function (F1="F")

 27 Sept. 2016   click/doubleclick Adr-Btn, prepare for HWREV_0_4 (not tested)
 08 Sept. 2016   using WiFi101 lib commit d27bf7c (fix for rssi=0)
 13 August 2016  hw-ddc-0.1 added with analog buttons for F0..F4
 06 August 2016  hw-rev 0.3 added, refactored addr selection and eeprom usage
 30 July 2016    added "wifi-lost" check
 29 July 2016    initial version for HWREV 0.2a

 if _DEBUG is defined, there MUST BE A TERMINAL connected to the

 *****************************************************************/
#define BUFFER_LENGTH  32

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <OneButton.h>
#include <AnalogButtons.h>
#include <Timer5.h>
//#include <Adafruit_SleepyDog.h>  // see https://github.com/arduino-libraries/WiFi101/issues/181

#include "pcb-type.h"
#include "FunkrEEPROM.h"
#include "Display7.h"
#include "dccutils.h"  // includes debug settings
#include "DCCLoco.h"
#include "RotaryEncoderMax.h"   // special version
#include "AddrSelection.h"

//*************** SW revision ***************************************
#define SW_REV_0_29
#define SW_STRING "SW_0.29"

//*************** lib used for 2-digit 7-segment display *************
Display7 disp;

//*************** define command control system **********************



//***** operating modes, changed by hitting the red "A" button *******
#define MODE_NORMAL               0
#define MODE_ADDRESS_SELECTION    1   // double press of "A" button
#define MODE_WAITING_FOR_RESPONSE 2   // waiting for loco info from central
#define MODE_CONFIG               3   // for storing new values in EEPROM
#define MODE_WAITING_FOR_WIFI     4   // not connected to wifi
#define MODE_FAST_ADDRESS_SELECTION  5   // short press of "A" button
#define MODE_ERROR_NO_CONNECTION  6   // error mode, there is no wifi connection

#define BLOCKING_TIME  2000   // don#t read feedback from SX Bus for 2 secs
// after sending new speed value

#define LIST_NETWORKS    // if defined, networks are scanned at start

//************** rotary decoder definition ********************
RotaryEncoderMax encoder(ENC1, ENC2);

AddrSelection addrSel;

//************** network constants *****************************
WiFiUDP Udp;                           // using UDP messages
const IPAddress lanbahnip(239, 200, 201, 250);  // lanbahn multicast IP address
const unsigned int lanbahnport = 27027;      // lanbahn port to listen on

//************** EEPROM ***************************************
FunkrEEPROM eep;

//************* Button declarations ***************************

#ifdef HWREV_D_0_1   // 4-digit display
OneButton addrBtn(ADDR_BTN, true);
OneButton stopBtn(STOP_BTN, true);
AnalogButtons analogButtons = AnalogButtons(ANALOG_BTN_PIN, INPUT, 2, 20);
Button f0Btn = Button(856, &f0Clicked);
Button f1Btn = Button(709, &f1Clicked);
Button f2Btn = Button(10, &f2Clicked, &switchOffBatt, 5000);
Button f3Btn = Button(522, &f3Clicked);
Button f4Btn = Button(335, &f4Clicked);
#elif defined (HWREV_0_4)   // 2-digit display, 2 function leds
OneButton stopBtn(STOP_BTN, true);
AnalogButtons analogButtons = AnalogButtons(ANALOG_BTN_PIN, INPUT, 2, 40);
Button f0Btn = Button(341, &f0Clicked);
Button f1Btn = Button(682, &f1Clicked, &switchOffBatt, 5000);
Button addrBtn = Button(10, &addrClicked);
#else  // 2 digit display
// no analog buttons
OneButton addrBtn(ADDR_BTN, true);
OneButton stopBtn(STOP_BTN, true);
OneButton f0Btn(F0_BTN, true);
OneButton f1Btn(F1_BTN, true);
#endif

bool changedFlag = false;  // if true, an update Message needs to be sent

char packetBuffer[255]; //buffer to hold incoming packet
#define BUF_LEN   80
char buffer[BUF_LEN];       // for message strings

int batteryVoltage;

//******* timers *********************************************
long switchOffTimer;
long announceTimer, blinkTimer, updateLocoTimer, updateTimer,
		lastLocoCommandSentTime, connectTimer;
long addrButtonTimer = 0;
long lastSpeedModTime =0;
long readFromCentral = 0;

long sentRequestTime = 0;  // time of last Request of LocoSpeed from central

// after start: wait if we have data about current loco
int mode = MODE_WAITING_FOR_RESPONSE;

uint16_t batteryLevel = 0;
long lastBatteryTimer = 0;

uint8_t trackPower = POWER_UNKNOWN;  // or POWER_OFF, POWER_ON
long encButtonTimer = 0;   // to check for LONG PRESS of encoder button
uint8_t encButtonPressed = 0;

int wifiRetries = 0;

// ************ network and other string variables, read from EEPROM later
String ssid;
String pass;

uint16_t ccmode = CCMODE_SX;

DCCLoco loco;   // construct DCC loco

uint16_t cal_33 = VOLT_3300;   // battery meas. cal constant
uint16_t myid = 0;

void setup() {

  
	// define frequency of interrupt
	MyTimer5.begin(IRQ_FREQ);  // 200=for toggle every 5msec => 2  digits
							   // 400 for 4 digit display to avoid flicker
	// define the interrupt callback function
	MyTimer5.attachInterrupt(display_irq);
	// start the timer
	MyTimer5.start();

	pinMode(BATT_ON, OUTPUT);
	digitalWrite(BATT_ON, HIGH);  // batt power on
	disp.setDecPoint(BW, true);

#ifdef HWREV_0_4     // enable function led outputs
	pinMode(LED_F0, OUTPUT);
	pinMode(LED_F1, OUTPUT);
#endif

#ifdef _DEBUG
	Serial.begin(57600); // USB is always 12 Mbit/sec
	long t1 = millis();
	while ((!Serial) && ((millis() - t1) < 10000)) {
		// make sure we read everything
		//BUT wait only for 20 secs
		// Watchdog.reset();   // no watchdog enabled so far
		delay(100);
	}

#endif

	// Initiliaze EEPROM library.
	eep.init();    // calling Wire.begin();

	// init buttons, ADC and Timers
	encoder.init();

	initButtons();

	analogReadResolution(10);

	disp.init();

	updateBuffer();

	announceTimer = millis(); // reset timer
	blinkTimer = millis(); // reset timer
	switchOffTimer = millis(); // resettimer
	connectTimer = millis();
#ifdef _DEBUG
  listNetworks();
#endif
	if (initFromEEPROM()) {
		// there have been wifi values in the EEPROM
		connectToWiFi();  // first connect
		disp.dispNumber(loco.getAddress());
		mode = MODE_WAITING_FOR_WIFI;
	} else {
		// go straight to error mode and wait for
    // config button press
		mode = MODE_ERROR_NO_CONNECTION;
		disp.dispCharacters('E', '2');
	}
	//Watchdog.enable(8192);
#ifdef _DEBUG
  Serial.println("DCC mode");
#endif
}

void initButtons() {

	// stop-button has a separate input for all HARDWARE revisions
	pinMode(STOP_BTN, INPUT_PULLUP);

#ifdef HWREV_D_0_1
	// functions
	analogButtons.add(f0Btn);
	analogButtons.add(f1Btn);
	analogButtons.add(f2Btn);
	analogButtons.add(f3Btn);
	analogButtons.add(f4Btn);

	// address button
	pinMode(ADDR_BTN, INPUT_PULLUP);
	addrBtn.setClickTicks(300);
	addrBtn.setPressTicks(5000);// 5 secs for long press => entering config mode
	addrBtn.attachClick(addrClicked);
	//addrBtn.attachDoubleClick(addrDoubleClicked);
	addrBtn.attachLongPressStart(toggleConfig);

#elif defined HWREV_0_4
	// functions
	analogButtons.add(f0Btn);
	analogButtons.add(f1Btn);

	// address button
	analogButtons.add(addrBtn);

#else
	// activate pullups
	pinMode(F0_BTN, INPUT_PULLUP);
	pinMode(F1_BTN, INPUT_PULLUP);
	pinMode(ADDR_BTN, INPUT_PULLUP);

	// functions
	f0Btn.attachClick(f0Clicked);
	f0Btn.setClickTicks(100);  // click detected after xx ms
	f1Btn.attachClick(f1Clicked);
	f1Btn.setClickTicks(100);
	f1Btn.setPressTicks(5000);  // 5 secs for long press => switch off
	f1Btn.attachLongPressStart(switchOffBatt);

	stopBtn.setClickTicks(100);

	// address button
	addrBtn.setClickTicks(300);
	addrBtn.setPressTicks(5000); // 5 secs for long press => entering config mode
	addrBtn.attachClick(addrClicked);
	addrBtn.attachDoubleClick(addrDoubleClicked);
	addrBtn.attachLongPressStart(toggleConfig);  
#endif

stopBtn.attachClick(stopClicked);    // all hardware revisions

}

/** get mode setting, the locoList and the currentLoco from the EEPROM
 *
 */
bool initFromEEPROM() {
#ifdef _DEBUG
	Serial.println("initFromEEPROM");
#endif
	// ********** read wifi params

	bool validWiFiParams;

	ssid = eep.readSSID();
	pass = eep.readPASS();

	if ((ssid.length() == 0) || (pass.length() == 0)) {
		validWiFiParams = false; // no values in EEPROM
	} else {
		validWiFiParams = true;
	}

	// *********** read last selected loco address
	uint16_t addr = addrSel.initFromEEPROM();
	loco.setAddress(addr);

	// ***** read A/D calibration constant
	cal_33 = eep.readVoltCalibration();
	if ((cal_33 < 3000) || (cal_33 > 3500)) {  //makes no sense
		// not yet set
		eep.writeVoltCalibration(VOLT_3300);
		cal_33 = VOLT_3300;
	}

	// ***** read ID
	myid = eep.readID();
	if (myid == 0xffff) {
		// not yet set
		myid = random(1000);
		eep.writeID(myid);
	}

#ifdef _DEBUG
	printConfig();
#endif

	return validWiFiParams;

}

void reconnectToWiFi() {

  // for now: RESET

  doReset();  //**********************************************

  return;
  
	// Connect to WPA/WPA2 network:
	WiFi.begin(ssid, pass);
	wifiRetries++;
	//Watchdog.reset();
	delay(1000);  // wait 1 seconds
	Udp.beginMulti(lanbahnip, lanbahnport);

	m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
	Serial.println("deep sleep ENABLED");

	Serial.print("trying reconnect, millis=");
	Serial.println(millis());
	Serial.print("#retries=");
	Serial.println(wifiRetries);

	// wait 2 seconds
	//Watchdog.reset();
	delay(2000);  // wait
	//Watchdog.reset();

	if (WiFi.status() != WL_CONNECTED) {
		mode = MODE_ERROR_NO_CONNECTION;
	} else {
	  mode = MODE_NORMAL;
	}

}

void connectToWiFi() {

#ifdef _DEBUG
	Serial.print("trying connect to ssid="); 
	Serial.print(ssid);
	Serial.print("  pass=");
	Serial.print(pass);
	Serial.print("  #=");
	Serial.println(wifiRetries);
#endif
#ifdef DIGITS4
	disp.blinkCharacters('1', '2', '3', '4');
#else
	disp.blinkCharacters('-', '-');
#endif
	static int connectCounter = 0;
	// attempt to connect to Wifi network:
	while ((WiFi.status() != WL_CONNECTED) && (connectCounter < 3)) {
		// Connect to WPA/WPA2 network:
		WiFi.begin(ssid, pass);
		wifiRetries++;
		//Watchdog.reset();
		delay(500);  // wait

		Udp.beginMulti(lanbahnip, lanbahnport);
		m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);

#ifdef _DEBUG
		Serial.println("deep sleep ENABLED");
		Serial.print("trying connect, millis=");
		Serial.println(millis());
		Serial.print("#retries=");
		Serial.println(wifiRetries);
#endif
		connectCounter++;

	}
	if (connectCounter >= 2) {
		disp.dispCharacters('E', '8');
#ifdef _DEBUG
		Serial.print("ERROR: could not connect to ");
		Serial.println(ssid);
#endif
		//Watchdog.reset();
		delay(100);

		mode = MODE_ERROR_NO_CONNECTION;


	}
	if (WiFi.status() == WL_CONNECTED) {
		disp.dispCharacters('0', '0');
#ifdef _DEBUG
		Serial.print("successfully connected to ");
		Serial.println(ssid);
#endif
	}
}

void updateBuffer() {
	IPAddress ip = WiFi.localIP();
	unsigned long secs = millis() / 1000;  // type of secs because of sprintf
	int rssi = WiFi.RSSI();
	// TODO remove noise from analogread (HW cap.?)
	batteryVoltage = (map(analogRead(BATT_PIN), 0, 1023, 0, cal_33)); // mV on the divider
	batteryVoltage = batteryVoltage * 2;                         //mV on Battery
	//batteryState = map(batteryVoltage , 3200, 4250, 0, 100);              //% of charge
	char rev[] = HW_STRING ":" SW_STRING;
	//int secs = (int) (millis() / 1000);
	sprintf(buffer, "A FUNKR%d %d.%d.%d.%d %d %d %s %lu %d\n", myid, ip[0], ip[1],
			ip[2], ip[3], batteryVoltage, rssi, rev, secs, wifiRetries);
}

void readUdp() {
	if (WiFi.status() != WL_CONNECTED)
		return; // >>>>>>

	int packetSize = Udp.parsePacket();
	if (packetSize)  // read packet
	{
		// read the packet into packetBufffer
		int len = Udp.read(packetBuffer, 255);
		if (len > 0)
			packetBuffer[len] = 0;
	}
}

void sendAnnounceMessage() {
  if (mode == MODE_ERROR_NO_CONNECTION) return;  // do nothing if in error mode
	static int counter = 0;
	if ((WiFi.status() == WL_CONNECTED) && (WiFi.RSSI() > -80)
			&& (WiFi.RSSI() != 0)) {
		// wifi is o.k., we can send a message
		updateBuffer();
		// send lanbahn announce packet
		Udp.beginPacket(lanbahnip, lanbahnport);
		Udp.write(buffer);
		Udp.endPacket();
		counter = 0;
	} else {

#ifdef _DEBUG
		Serial.println("ERROR: wifi connection lost, rssi low or zero");
		Serial.print("rss=");
		Serial.println(WiFi.RSSI());
#endif
		counter++;
		if (counter >= 3) {
			reconnectToWiFi();
      counter = 0;
		}
	}
}

// update the display every 5msec
// alternate between left and right display (or all 4 digits in case of 4-digit display
void display_irq(void) {
	static int d = 0;
#ifdef DIGITS4
	d++;
	if (d >= 4)
	d = 0;
#else
	if (d == 0) {
		d = 1;
	} else {
		d = 0;
	}
#endif
	disp.doDisplay(d);
}

/** start address selection mode when address button is clicked
 *   release address mode when clicked again
 *   uses only the address list stored in EEPROM
 */
void addrClicked() {
#ifdef HWREV_0_4
	// TODO: test
	static long lastClicked = 0;
	if (( millis() - lastClicked) < 300) {
		// this button was already clicked during the last 300msec
		// =>> consider as double clicked
		if (mode == MODE_FAST_ADDRESS_SELECTION) {
			mode = MODE_ADDRESS_SELECTION;
			// restart selection
			addrSel.start(loco.getAddress(),true);// select from all addresses
			// 1..99
		}
	}
	lastClicked = millis();
#endif

#ifdef _DEBUG
	Serial.println("addr button clicked.");
#endif
	uint16_t newAddress;
	// toggle address/normal mode
	switch (mode) {
	case MODE_NORMAL:
		mode = MODE_FAST_ADDRESS_SELECTION;
		addrSel.start(loco.getAddress(), false);   // multi-address mode
		// addresses from locolist only
		break;

	case MODE_FAST_ADDRESS_SELECTION:
	case MODE_ADDRESS_SELECTION:
		addrClickEnd();
		break;

	case MODE_WAITING_FOR_RESPONSE:
	case MODE_WAITING_FOR_WIFI:
#ifdef _DEBUG
		Serial.println("in waiting mode. do nothing.");
#endif
		// do nothing.
		break;

	}
	switchOffTimer = millis();  // reset switch off timer
}

void addrClickEnd() {
	uint16_t newAddress = addrSel.end();
	if (newAddress != loco.getAddress()) {  // new loco
		loco.setAddress(newAddress);
    mode = MODE_NORMAL;
    disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());

	} else { // no change, just continue with stored value for current speed
		encoder.setPosition(loco.getSpeed());
		mode = MODE_NORMAL;
		disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());
	}
}

void sendAndDisplaySpeed() {
	if (WiFi.status() != WL_CONNECTED) {
#ifdef _DEBUG
		Serial.println("ERROR: wifi connection lost");
#endif
    return;
		// connectToWiFi();
	}

  // address first
  uint16_t addr = loco.getAddress();
  uint16_t abssp = loco.getAbsSpeed();
  uint8_t dir = loco.getBackward();
  String func =loco.getF0F4String();
  sprintf(buffer, "DCC1 %d %d %d %s\n", addr, abssp, dir, func.c_str());


#ifdef _DEBUG
	Serial.print(buffer);  // contains an "\n" already
#endif

	// send lanbahn announce packet
	Udp.beginPacket(lanbahnip, lanbahnport);
	Udp.write(buffer);
	Udp.endPacket();

	disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());

	lastLocoCommandSentTime = millis();  // loco command was updated
}

/** request loco info from interface
 * after this request, we are waiting for a second
 * to get a response from central - then setting
 * loco to "zero" if no response
 */
void sendRequestLoco(uint8_t addr) {
	sprintf(buffer, "R %d\n", addr);

#ifdef _DEBUG
	Serial.print(buffer);  // contains an "\n" already
#endif
	// send lanbahn announce packet
	Udp.beginPacket(lanbahnip, lanbahnport);
	Udp.write(buffer);
	Udp.endPacket();

	sentRequestTime = millis();
}

/** read incoming commands from Wifi
 *  "P 0" (= track power off) - or "P 1"
 *  "44 134" (=loco addr 44 has sx data=134)
 */
void interpretCommand(String s) {
	if ((s.charAt(0) != 'f') && (s.charAt(0) != 'F'))
		return; // only feedback cmd read

	// loco or trackpower feedback command
	// example:  "F44 31" = loco #44 has SX data= 31,
	//       (standard selectrix data format, 8 bit)
	s = s.substring(1);  // remove "F "
	int n = s.indexOf(' '); // command split by space,i.e. 2 numbers
	if (n != -1) {
		int i_ch = s.toInt();
		String s2 = s.substring(n + 1, s.length());
		int i_value = s2.toInt();
		if (i_ch == 127) {
			// trackpower command on channel 127
			if (i_value == 0) {
				trackPower = POWER_OFF;
			} else {
				trackPower = POWER_ON;
			}
		}
	}
    //TODO loco state request from central station 
}

void functionClicked(uint8_t index) {
#ifdef _DEBUG
	Serial.print("F");
	Serial.print(index);
	Serial.println(" clicked.");
#endif
	if (mode == MODE_NORMAL) {  // ignore in other modes
		uint8_t val = loco.toggleFunction(index);
#ifdef HWREV_D_0_1
		disp.dispCharacters('F', '0' + index, '=', '0' + val, 1000);
#endif
#ifdef HWREV_0_4
		digitalWrite(LED_F0, loco.getF0());
		digitalWrite(LED_F1, loco.getF1());

#endif
		changedFlag = true;
	}
	userInteraction();  // reset switch off timer
}
void f0Clicked() {
	functionClicked(0);
}

void f1Clicked() {
	functionClicked(1);
}

void f2Clicked() {
	functionClicked(2);
}

void f3Clicked() {
	functionClicked(3);
}

void f4Clicked() {
	functionClicked(4);
}

void stopClicked() {
#ifdef _DEBUG
	Serial.println("Stop clicked.");
#endif
	if (mode == MODE_NORMAL) {
		loco.stop();  // does not change direction!
		changedFlag = true;
		encoder.setPosition(0);
	}
	userInteraction();  // reset switch off timer
}

//void stopLongClicked() {
	// TODO
//}

/** parse the list of locos
 *  from input String
 *  format:    10,44,64  (use locos with address 10, 44 or 64)
 *  format:    10,a  (last loco was 10, use 10, all other addresses are selectable)
 */

void sendTrackPower() {
	// send value of trackpower on channel 127
	if (trackPower == POWER_OFF) {
		sprintf(buffer, "S 127 0");
	} else if (trackPower == POWER_ON) {
		sprintf(buffer, "S 127 1");
	}  // do nothing when power state not known

#ifdef _DEBUG
	Serial.print(buffer);  // contains an "\n" already
#endif

	// send lanbahn packet
	Udp.beginPacket(lanbahnip, lanbahnport);
	Udp.write(buffer);
	Udp.endPacket();

}

void loop() {
	//Watchdog.reset();

	encoder.tick();
	stopBtn.tick();
	addrBtn.tick();
#ifdef HWREV_D_0_1
	analogButtons.check();
#else
	f0Btn.tick();
	f1Btn.tick();
#endif

	readUdp();


  if (mode == MODE_ERROR_NO_CONNECTION) {
    // do nothing, just wait for button input (to switch off
    // or to switch into config mode
    delay(100);
    disp.dispCharacters('E', '7');
  } else if (mode == MODE_WAITING_FOR_WIFI) {
		delay(100);
		if (WiFi.status() == WL_CONNECTED) {
			mode = MODE_NORMAL;
			disp.dispNumber(loco.getAddress());
#ifdef _DEBUG
			Serial.println("switching from waiting to normal");
#endif
		}
	} else if ((mode == MODE_ADDRESS_SELECTION)
			|| (mode == MODE_FAST_ADDRESS_SELECTION)) {
		// selecting a new address for the loco
		// DO NOT SEND Loco messages during the selection
		if ((millis() - updateTimer) > 100) {
			updateTimer = millis();
			// SX only / boolean single = (mode == MODE_ADDRESS_SELECTION);
			addrSel.doLoop(false);  // always select from already existing list
		}
	} else if (mode == MODE_NORMAL) {
  
    // send the announce string every 20sec
    if ((millis() - announceTimer) >= 20000) {
      announceTimer = millis();
      sendAnnounceMessage();
#ifdef _DEBUG
      Serial.println(buffer);
#endif
    }
    
		// send loco command refresh at least every 5 secs also without user interaction
		if (((millis() - updateLocoTimer) > 5000)) {
			updateLocoTimer = millis();
			sendAndDisplaySpeed();  // dispSpeed() will also be called.
			changedFlag = false;
		}

		// every 100ms check encoder for speed/functions change
		if ((millis() - updateTimer) >= 50) {
			updateTimer = millis();

			long newSpeed;
			newSpeed = encoder.getPositionMax(MAX_SPEED);
      int16_t actSpeed = loco.getSpeed();
			if (newSpeed != actSpeed) {
				/* have a "zero with sign" i.e. stop the loco first
				 without changing the direction, then change the direction
				 but leave speed at 0, then normal speed setting */
				if ((newSpeed < 0) && (loco.getBackward() == 0)) {
					encoder.setPosition(0);
					loco.setSpeed(0);
					loco.setBackward(true);
				} else if ((newSpeed > 0) && (loco.getBackward() != 0)) {
					encoder.setPosition(0);
					loco.setSpeed(0);
					loco.setBackward(false);
				} else if (newSpeed == 0) {
					loco.stop();
				} else {
          if ( 
               ((millis() - lastSpeedModTime) < 100) 
               && (abs(actSpeed) >= 5) 
             ) {
            if (newSpeed > actSpeed) {
              newSpeed += 10;
              if (newSpeed > MAX_SPEED) newSpeed = MAX_SPEED;
            } else {
              newSpeed -= 10;
              if (newSpeed < -MAX_SPEED) newSpeed = -MAX_SPEED;
            }          
          }
                 
					loco.setSpeed(newSpeed);
          encoder.setPosition(newSpeed);
				}
				changedFlag = true;
       lastSpeedModTime = millis();
			}
			if (changedFlag == true) { // there was some user interaction
				userInteraction();  // reset switch off timer
				sendAndDisplaySpeed();  // dispSpeed() will also be called.
				// reset changed flag
				changedFlag = false;
			}
		}

	} else if (mode == MODE_WAITING_FOR_RESPONSE) {
		//disp.blinkCharacters('-','-');
		// check for timeout for waiting for response
		if ((millis() - sentRequestTime) > 2000) {
#ifdef SX
			loco.setFromSXData(0); // reset loco
#endif
#ifdef DCC
      loco.init();
#endif
			mode = MODE_NORMAL;
			encoder.setPosition(0); // reset encoder position to 0
			disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());
		}
	} else if (mode == MODE_CONFIG) {
		// go into config mode
		disp.dispCharacters('c', 'o'); // == (co)nfig
		delay(20);
		long timeout = millis();
		if (!Serial)
			Serial.begin(57600);
		while (!Serial && ((millis() - timeout) < 20000)) {
			// wait for 20 seconds for serial usb connection.
			//Watchdog.reset();
			delay(100);
		}
		if (!Serial) {  // no sense to wait any longer
			mode = MODE_NORMAL;
			disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());
		} else {
			if (Serial.available() > 0) {
				String line = Serial.readStringUntil('\n'); // has 1 second timeout
				// line.toLowerCase(); SSID is case sensitive
				userInteraction(); // reset switch off timer
				//Watchdog.reset();
				bool end = updateConfig(line);
				if (end) {
					mode = MODE_NORMAL;
					disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());
					Serial.println("exiting config");
#ifndef _DEBUG
					Serial.end();
#endif
				}

			}
		}

	} else {
		// something went wrong
		mode = MODE_NORMAL;
#ifdef _DEBUG
		Serial.println("unknown mode - switched back to NORMAL");
#endif
	}

	// switch off after 10 minutes w/o user interaction
	if ((millis() - switchOffTimer) >= 10 * 60 * 1000) {
		switchOffBatt(); // ==>> THIS IS THE END.
	}

}

void switchOffBatt() {
	// switch off
	digitalWrite(BATT_ON, LOW); // ==>> THIS IS THE END.

}
/** check if input line selects a valid cc value
 *  return INVALID, if not possible
 *  return ccmode (int), if possible
 */

/*int isValidCCMode(String s) {
 if (s.indexOf('=') != -1) {  // should be not necessary
 String val = s.substring(s.indexOf('=') + 1, s.indexOf('\n'));
 String newcc = s_ccmode[ccmode];
 for (int i = 0; i < N_CCMODE; i++) {
 if (val.equalsIgnoreCase(s_ccmode[i])) {
 ccmode = i;
 return ccmode;
 }
 }
 }
 return INVALID;
 }  */

void toggleConfig(void) {
#ifdef DEBUG
	Serial.print("toggleConfig, mode=");
	Serial.println(mode);
#endif
	if (mode == MODE_CONFIG) {
		Serial.println("ending Config");
		mode = MODE_NORMAL;
		disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());
#ifndef _DEBUG
		Serial.end();
#endif
	} else {
		disp.dispCharacters('c', 'o'); // == (co)nfig
		if (!Serial) { //  config only works with USB connected
			uint32_t timeout = millis();

			Serial.begin(57600); // must initialize when not in debug
			while (!Serial && ((millis() - timeout) < 10000)) { // 10 secs timeout
				delay(100);
				//Watchdog.reset();
			}
			if (!Serial) {
				// timeout, nothing connected on USB, go back to normal mode
				mode = MODE_NORMAL;
				disp.dispNumberSigned(loco.getSpeed(), loco.getBackward());
				return;  // ===>>> back to normal
			}
		}
		printConfigHelp();
		mode = MODE_CONFIG;
	}
}

void printWifi(void) {
	Serial.print("WiFi.status()=");
	int stat = WiFi.status();
	switch (stat) {
	case WL_NO_SHIELD:
		Serial.println("WL_NO_SHIELD");
		break;
	case WL_IDLE_STATUS:
		Serial.println("WL_IDLE_STATUS");
		break;
	case WL_NO_SSID_AVAIL:
		Serial.println("WL_NO_SSID_AVAIL");
		break;
	case WL_SCAN_COMPLETED:
		Serial.println("WL_SCAN_COMPLETED");
		break;
	case WL_CONNECTED:
		Serial.println("WL_CONNECTED");
		break;
	case WL_CONNECT_FAILED:
		Serial.println("WL_CONNECT_FAILED");
		break;
	case WL_CONNECTION_LOST:
		Serial.println(" WL_CONNECTION_LOST");
		break;
	case WL_DISCONNECTED:
		Serial.println(" WL_DISCONNECTED");
		break;
	default:
		Serial.println(stat);
	}

	Serial.print("WiFi.RSSI()=");
	Serial.println(WiFi.RSSI());
}

void printConfig(void) {
	Serial.println("current config settings ----------------------");

	Serial.print("hw=");
	Serial.print(HW_STRING);
	Serial.print("  sw=");
	Serial.print(SW_STRING);
	Serial.print("  myid=");
	Serial.println(myid);

	Serial.print("ssid=");
	Serial.print(ssid);
	Serial.println("  pass=******");

	Serial.print("locoAdr=");
	uint16_t a = loco.getAddress();
	Serial.print(a);
	Serial.print(" ");

	Serial.print("eepLastLoco=");
	uint16_t last = eep.readLastLoco();
	Serial.println(last);

	Serial.print("  locos=");
	String ll = addrSel.getLocos();
	ll.trim();
	Serial.print(ll);
	//Serial.print("  cc=");
	//String s = s_ccmode[ccmode];
	//Serial.println(s);

	Serial.print("  batt-cal=");
	Serial.println(cal_33);
	Serial.println("----------------------------------------------");
}

void printConfigHelp(void) {
	Serial.println(
	//	"CONFIGURATION - commands: exit, list, ssid=, pass=, cc= , locos=, addrmode=");
			"CONFIGURATION - commands: exit, list, ssid=, pass=, locos=");
	Serial.println("debug commands are: wifi, reconnect");
	Serial.println("command ends with newline");
	printConfig();
	Serial.println("command? (end with NL)");
}

void userInteraction() {
	switchOffTimer = millis();  // reset switch off timer
}

/** returns true if "exit" was selected
 *
 */
bool updateConfig(String line) {
	Serial.print("parsing line: ");
	Serial.println(line);
	if (line.startsWith("exit")) {
		// close config (can also be done by ADDR_BTN longPress)
		return true;
	}

	if (line.startsWith("ssid=")) {
		String s = line.substring(line.indexOf('=') + 1);
		if (eep.writeSSID(s)) {
			ssid = s;
			Serial.print("ssid=");
			Serial.print(s);
			Serial.println(":");
		} else {
			Serial.print("could not write config");
		}
	} else if (line.startsWith("pass=")) {
		String s = line.substring(line.indexOf('=') + 1);
		if (eep.writePASS(s)) {
			pass = s;
			Serial.print("pass=:");
			Serial.print(s);
			Serial.println(":");
		} else {
			Serial.print("could not write config");
		}
	} else if (line.startsWith("wifi")) {
		printWifi();
	} else if (line.startsWith("reconnect")) {
		reconnectToWiFi();
	} else if (line.startsWith("locos=")) {
		String s = line.substring(line.indexOf('=') + 1);
		if (eep.writeLocoList(s)) {
			Serial.print("locos=");
			Serial.println(s);
		} else {
			Serial.println("could not write config");
		}
	} else if (line.startsWith("list")) {
		printConfigHelp();
	}
	return false; // not the end of configuration
}

void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1)
  {
    Serial.println("Couldn't get a wifi connection");
    while (true);
  }

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));
    Serial.flush();
  }
}

void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.println("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.println("Auto");
      break;
  }
}

void doReset(void) {
   Serial.println("Performing reset");
  // Watchdog.enable(16);
   delay(50);
}
// ******************************************** snippets ********************************************
/* TODO TRACKPOWER ON/off
 if ( encButtonPressed
 && ((millis() - encButtonTimer) > 1300 )
 && (trackPower != POWER_UNKNOWN) ) {
 // toggle trackPower (only when in known state)
 if (trackPower == POWER_ON) {
 trackPower = POWER_OFF;
 } else {
 trackPower = POWER_ON;
 }
 sendTrackPower();
 encButtonPressed = 0;  // do not toggle again
 #ifdef _DEBUG
 Serial.println("toggled trackpower.");
 #endif
 }  // endif encButtonTimer  */
