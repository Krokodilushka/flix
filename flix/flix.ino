// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include <NimBLEDevice.h>
#include <NimBLEAddress.h>

#define SERIAL_BAUDRATE 115200
#define WIFI_ENABLED 1
#define RC_ENABLED 1
#define GAMEPAD_ENABLED 0

double t = NAN; // current step time, s
float dt; // time delta from previous step, s
int16_t channels[16]; // raw rc channels
float controls[16]; // normalized controls in range [-1..1] ([0..1] for throttle)
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [0..1]

void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	print("Initializing flix\n");
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
#if RC_ENABLED
	setupRC();
#endif
#if GAMEPAD_ENABLED
	setupGamepad();
#endif
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
	readIMU();
	step();
#if RC_ENABLED
	readRC();
#endif
#if GAMEPAD_ENABLED
	loopGamepad();
#endif
	estimate();
	control();
	sendMotors();
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters();
}
