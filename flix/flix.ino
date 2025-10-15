// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include <Wire.h>
#include "util.h"
#include <NimBLEDevice.h>
#include <NimBLEAddress.h>
#include "config.h"
#include <SPI.h>

#define SERIAL_BAUDRATE 115200

double t = NAN; // current step time, s
float dt; // time delta from previous step, s
float controlRoll, controlPitch, controlYaw, controlThrottle; // pilot's inputs, range [-1, 1]
float controlMode = NAN;
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [0..1]

void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SS_PIN);
	Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
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
#if BATTERY_ENABLED
	setupBattery();
#endif
#if BAROMETER_ENABLED
	setupBarometr();
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
#if BATTERY_ENABLED
	loopBattery();
#endif
#if BAROMETER_ENABLED
	loopBarometr();
#endif
}
