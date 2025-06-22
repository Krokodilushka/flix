// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Utility functions

#pragma once

#include <math.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>

const float ONE_G = 9.80665;

float mapf(long x, long in_min, long in_max, float out_min, float out_max) {
	return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float mapff(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Wrap angle to [-PI, PI)
float wrapAngle(float angle) {
	angle = fmodf(angle, 2 * PI);
	if (angle > PI) {
		angle -= 2 * PI;
	} else if (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}

// Disable reset on low voltage
void disableBrownOut() {
	REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
}

// Trim and split string by spaces
void splitString(String& str, String& token0, String& token1, String& token2) {
	str.trim();
	char chars[str.length() + 1];
	str.toCharArray(chars, str.length() + 1);
	token0 = strtok(chars, " ");
	token1 = strtok(NULL, " "); // String(NULL) creates empty string
	token2 = strtok(NULL, "");
}

bool readRegister(TwoWire *wire, int device, int reg, uint8_t *buffer, size_t length) {
    if (buffer == nullptr || length == 0)
    {
        Serial.print("Пустой буфер или длина = 0");
        return false;
    }

    wire->beginTransmission(device);
    wire->write(reg);
    if (wire->endTransmission(false) != 0)
    {
        Serial.printf("Ошибка передачи адреса регистра 0x%02X для устройства 0x%02X\n", reg, device);
        return false;
    }

    size_t bytesRead = wire->requestFrom(device, length);
    if (bytesRead == length)
    {
        for (size_t i = 0; i < length; ++i)
        {
            buffer[i] = wire->read();
        }
        return true;
    }
    else
    {
        Serial.print("Ошибка запроса данных\n");
        return false;
    }
}