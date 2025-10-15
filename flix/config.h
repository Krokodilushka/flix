#pragma once
#include <WiFi.h>

#define WIFI_ENABLED 1
#define WIFI_SSID "Xiaomi Pad 6"
#define WIFI_PASSWORD "bg575web87n54bs"
#define WIFI_MODE WIFI_MODE_STA
#define WIFI_TX_POWER WIFI_POWER_7dBm

#define RC_ENABLED 0

#define GAMEPAD_ENABLED 1
#define GAMEPAD_THROTTLE_CHANGE_INTERVAL 0.01f // С каким интервалом регулировать тягу (10 мс)
#define GAMEPAD_STICK_L_DEADZONE 0.05f         // Не реагировать на небольшие движения левого стика
#define GAMEPAD_RSSI_INTERVAL 1.0

#define USE_DSHOT 0 // Включить цифровой протокол DShot для ESC с бесколлекторными моторами
#define MOTOR_0_PIN GPIO_NUM_2
#define MOTOR_1_PIN GPIO_NUM_6
#define MOTOR_2_PIN GPIO_NUM_11
#define MOTOR_3_PIN GPIO_NUM_21
// #define MOTOR_3_PINGPIO_NUM_1
// #define MOTOR_3_PIN GPIO_NUM_42
// #define MOTOR_3_PIN GPIO_NUM_41
// #define MOTOR_3_PIN GPIO_NUM_2

#define I2C_SDA_PIN GPIO_NUM_5
#define I2C_SCL_PIN GPIO_NUM_4

#define SPI_SCK_PIN GPIO_NUM_35
#define SPI_MISO_PIN GPIO_NUM_37
#define SPI_MOSI_PIN GPIO_NUM_36
#define SPI_SS_PIN GPIO_NUM_38

#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 12

#define BATTERY_ENABLED 0
#define BATTERY_UPDATE_INTERVAL 0.1                          // Интервал обновления данных о батарее в секундах
#define BATTERY_CELLS 1                                      // Сколько ячеек аккумулятора (S1, S2...)
#define BATTERY_MIN_LEVEL 3400 * BATTERY_CELLS               // Минимальное безопасное напряжение аккумулятора (мВ)
#define BATTERY_MAX_LEVEL 4200 * BATTERY_CELLS               // Максимальное напряжение аккумулятора (мВ)
#define BATTERY_DISARMED_VOLTAGE_THRESHOLD BATTERY_MIN_LEVEL // Порог автоматического отключения моторов (мВ)
#define BATTERY_DISARMED_CURRENT_THRESHOLD_MIN 10            // Мин значение тока (mA) ниже которого моторы отключатся
#define BATTERY_DISARMED_CURRENT_THRESHOLD_MAX 15000         // Макс значение тока (mA) выше которого моторы отключатся
// Уровни напряжения для уведомлений в QGroundControl
#define BATTERY_MAVLINK_STATUS_LOW 3500 * BATTERY_CELLS
#define BATTERY_MAVLINK_STATUS_CRITICAL 3300 * BATTERY_CELLS
#define BATTERY_MAVLINK_STATUS_EMERGENCY BATTERY_MIN_LEVEL
#define INA226_RESISTOR_OHMS 0.005 // Значение резистора шунта (в Омах), используемого в INA226.
#define INA226_MAX_CURRENT_AMPS 15

#define BAROMETER_ENABLED 0
#define BAROMETER_UPDATE_INTERVAL 1.0 // С каким интервалом обновлять показания в секундах