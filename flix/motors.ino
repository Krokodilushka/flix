// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Motors output control using MOSFETs
// In case of using ESCs, change PWM_STOP, PWM_MIN and PWM_MAX to appropriate values in μs, decrease PWM_FREQUENCY (to 400)

#if USE_DSHOT
#include <DShotRMT.h>
#endif
#include "util.h"

#if USE_DSHOT
#define DSHOT_STOP 0
#define DSHOT_MODE DSHOT300
#define IS_BIDIRECTIONAL false
DShotRMT motor0(MOTOR_0_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor1(MOTOR_1_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor2(MOTOR_2_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor3(MOTOR_3_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
// Минимальная и максимальная мощность мотора в диапазоне 48–2047 (0–47 зарезервированы для специальных команд DShot).
// DSHOT_MAX можно установить меньше 2047, чтобы ограничить максимальную мощность моторов.
#define DSHOT_MIN 48
#define DSHOT_MAX 1000
#else
#define PWM_STOP 0
#define PWM_MIN 0
#define PWM_MAX 1000000 / PWM_FREQUENCY
#endif

// Motors array indexes:
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;

void setupMotors()
{
	print("Setup Motors\n");

#if USE_DSHOT
	motor0.begin();
	motor1.begin();
	motor2.begin();
	motor3.begin();
#else
	// configure pins
	ledcAttach(MOTOR_0_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttach(MOTOR_1_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttach(MOTOR_2_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttach(MOTOR_3_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
#endif

	sendMotors();
	print("Motors initialized\n");
}

#if !USE_DSHOT
int getDutyCycle(float value)
{
	value = constrain(value, 0, 1);
	float pwm = mapff(value, 0, 1, PWM_MIN, PWM_MAX);
	if (value == 0)
		pwm = PWM_STOP;
	float duty = mapff(pwm, 0, 1000000 / PWM_FREQUENCY, 0, (1 << PWM_RESOLUTION) - 1);
	return round(duty);
}
#endif

#if USE_DSHOT
int getDshotValue(float value)
{
	float dshotValue = mapff(value, 0, 1, DSHOT_MIN, DSHOT_MAX);
	if (value == 0)
		dshotValue = DSHOT_STOP;
	return round(dshotValue);
}
#endif

void sendMotors()
{
#if USE_DSHOT
	motor0.setThrottle(getDshotValue(motors[0]));
	motor1.setThrottle(getDshotValue(motors[1]));
	motor2.setThrottle(getDshotValue(motors[2]));
	motor3.setThrottle(getDshotValue(motors[3]));
#else
	ledcWrite(MOTOR_0_PIN, getDutyCycle(motors[0]));
	ledcWrite(MOTOR_1_PIN, getDutyCycle(motors[1]));
	ledcWrite(MOTOR_2_PIN, getDutyCycle(motors[2]));
	ledcWrite(MOTOR_3_PIN, getDutyCycle(motors[3]));
#endif
}

bool motorsActive()
{
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n)
{
	print("Testing motor %d\n", n);
	motors[n] = 1;
	delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	print("Done\n");
}
