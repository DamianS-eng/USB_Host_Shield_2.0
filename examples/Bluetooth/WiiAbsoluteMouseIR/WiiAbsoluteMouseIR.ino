/*
X and Y in absolute mouse mode start in upper left corner.
*/

#include <Wii.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#ifndef WIICAMERA // Used to check if WIICAMERA is defined
#error "Please set ENABLE_WII_IR_CAMERA to 1 in settings.h"
#endif

#include "HID-Project.h"
//#include <ProTrinketMouse.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
WII Wii(&Btd); // After the Wiimote pairs once, you can simply create the instance like so and re upload and then press any button on the Wiimote


const int switchPin = 9; 		//swich for mouse control
const int mouseButton = 10;
const int xAxis = 1;
const int yAxis = 0;
const int LedPin = 13;
//const int LedPin = LED_BUILDIN;

int range = 12;
int responseDelay = 5;			//global delay for mouse input
int threshold = range/4;		//deadzone
int center = range/2;
int lastReading = 1;
const float powerValue = 1.4; 	//used for exponential mouse acceleration
int16_t xBound = -32768;
int16_t yBound = 32767;
int16_t xIRMax = -1024;
int16_t yIRMax = 1023;


bool mouseIsActive = false;
bool lastSwitchState = false;
bool mouseButtonPress = false;
bool mouseAccel = false;

long debounceTime = 0;
long debounce = 50;

bool isWiiNotPointing(WII Wiimote) {
	// Only print if the IR camera is actually seeing something
	bool isIR1exist = (Wiimote.getIRx1() == 0x3FF || Wiimote.getIRy1() == 0x3FF || Wiimote.getIRs1() == 0)
	bool isIR2exist = (Wiimote.getIRx2() == 0x3FF || Wiimote.getIRy2() == 0x3FF || Wiimote.getIRs2() == 0)
	if (!isIR1exist || !isIR2exist) { 
		return true;
	}
	return false;
}

void mouseAcceleration(int Reading){
	return ((Reading > 0 ) ? (int)pow(powerValue, Reading) : -(int)pow(powerValue, -Reading));
}

int readAxis(int thisAxis) {
	int reading = analogRead(thisAxis);
	reading = map(reading, 0, 1023, 0, range);
	int distance = center - reading;
	if(abs(distance) < theshold) {
		distance = 0;
	}
	return distance;
}

void axisToMouse() {
	switchState = digitalRead(switchPin);
	if (switchState != lastSwitchState) {
		if (switchState) {
			mouseIsActive = !mouseIsActive;
		} 
	}
	lastSwitchState = switchState;
	
	xreading = readAxis(xAxis);
	yreading = readAxis(yAxis);
	if(mouseAccel) {
		xreading = mouseAcceleration(xreading);
		yreading = mouseAcceleration(yreading);
	}
	digitalRead(mouseButton);
	if(buttonReading != lastReading) {
		debounceTime = millis();
	}
	if((millis() - debounceTime) > debounce) {
		buttonState = buttonReading;
		mouseButtonPressed = !buttonState;
	}
	lastReading = buttonReading;
	digitalWrite(ledPin, mouseButtonPressed);
	if(mouseIsActive) {
		TrinketMouse.move(xReading, yReading, 0, MOUSEBTN_LEFT_MASK);
	} else {
		TrinketMouse.move(xReading, yReading, 0, 0);
	}
}

void setup() {
	Serial.being(115200);		//for USB communication
	
	pinMode(switchPin, INPUT_PULLUP);
	pinMode(mouseButton, INPUT_PULLUP);
	pinMode(LedPin, OUTPUT);
	
	/*
	if (digitalRead(pairButton) == HIGH) {
		WII Wii(&Btd, PAIR); // This will start an inquiry and then pair with your Wiimote - wire a button on which it will check at startup to perform
	}
	*/
	#if !defined(__MIPSEL__)
		while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
	#endif
	if (Usb.Init() == -1) {
		//Serial.print(F("\r\nOSC did not start"));
		while (1) { //halt
			digitalWrite(LedPin, lastSwitchState);
			lastSwitchState = !lastSwitchState;
			delay(200);
		}
	}
	//Serial.print(F("\r\nWiimote Bluetooth Library Started"));
	//TrinketMouse.begin();
	AbsoluteMouse.begin();
}

void loop() {
	int switchState;
	int buttonState;
	int xreading, yreading;
	int buttonReading;
	uint8_t batteryLevel = 0;
	
	Usb.Task();
	if (Wii.wiimoteConnected) {
		batteryLevel = Wii.getBatteryLevel()
		if (Wii.getButtonClick(HOME)) { // You can use getButtonPress to see if the button is held down
			Wii.disconnect();
		}
		if (!Wii.isIRCameraEnabled() && Wii.getButtonClick(A)) {
			Wii.IRinitialize();
		}
		if (isWiiNotPointing(WII Wii)) {
			xreading = 0;
			yreading = 0;
		} else {
      // Arduino Map function to covert one range to another
      // https://www.arduino.cc/reference/en/language/functions/math/map/
      // map(value, fromLow, fromHigh, toLow, toHigh)			
			xreading = ((Wii.getIRx1() + Wii.getIRx2()) / 2) * xBound / xIRMax;
			yreading = ((Wii.getIRy1() + Wii.getIRy2()) / 2 ) * yBound / yIRMax;
		}
	}
	AbsoluteMouse.moveTo(xreading, yreading);
	delay(responseDelay);
}