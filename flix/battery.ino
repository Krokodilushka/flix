/*
 * Считывает данные с датчика тока INA226 с интервалом BATTERY_UPDATE_INTERVAL.
 * При падении напряжения ниже BATTERY_DISARMED_THRESHOLD происходит автоматическое отключение моторов.
 * Показания тока и напряжения передаются в QGroundControl (см. файл mavlink.ino).
 * В QGroundControl отображается иконка батареи, ток и напряжение можно вывести в виде виджета.
 * Можно настроить автоматическое отлючение моторов при высоком и низком токе.
 * Отключение при высоком токе может быть полезено, если моторы по какой то причине включатся на максимальную тягу.
 * Отключение при низком токе может быть полезено, если нет данных от датчика тока.
 */

extern bool armed; // Нужно для отключения моторов при низком заряде

#define BATTERY_UPDATE_INTERVAL 0.1                          // Интервал обновления данных о батарее в секундах
#define BATTERY_CELLS 4                                      // Сколько ячеек аккумулятора (S1, S2...)
#define BATTERY_MIN_LEVEL 3400 * BATTERY_CELLS               // Минимальное безопасное напряжение аккумулятора (мВ)
#define BATTERY_MAX_LEVEL 4200 * BATTERY_CELLS               // Максимальное напряжение аккумулятора (мВ)
#define BATTERY_DISARMED_VOLTAGE_THRESHOLD BATTERY_MIN_LEVEL // Порог автоматического отключения моторов (мВ)
#define BATTERY_DISARMED_CURRENT_THRESHOLD_MIN 10            // Мин значение тока (mA) ниже которого моторы отключатся
#define BATTERY_DISARMED_CURRENT_THRESHOLD_MAX 15000         // Макс значение тока (mA) выше которого моторы отключатся

// Уровни напряжения для уведомлений в QGroundControl
#define BATTERY_MAVLINK_STATUS_LOW 3500 * BATTERY_CELLS
#define BATTERY_MAVLINK_STATUS_CRITICAL 3300 * BATTERY_CELLS
#define BATTERY_MAVLINK_STATUS_EMERGENCY BATTERY_MIN_LEVEL

#define INA226_RESISTOR_OHMS 0.005                                                                   // Значение резистора шунта (в Омах), используемого в INA226.
#define INA226_MAX_CURRENT_AMPS 15                                                                   // Максимальный измеряемый ток (в амперах), для расчёта масштабирующего коэффициента (LSB)
#define INA226_CURRENT_LSB INA226_MAX_CURRENT_AMPS / 32768.0                                         // Точность тока на 1 бит
#define INA226_POWER_LSB 25 * INA226_CURRENT_LSB                                                     // Точность мощности, см. документацию INA226
#define INA226_CURRENT_CALIBRATION (uint16_t)(0.00512 / (INA226_CURRENT_LSB * INA226_RESISTOR_OHMS)) // Коэффициент калибровки

// Адреса регистров датчика
#define INA226_DEVICE_ADDRESS 0x40
#define INA226_REG_BUS_VOLTAGE 0x02
// #define INA226_REG_POWER 0x03
#define INA226_REG_CURRENT 0x04
#define INA226_REG_CALIBRATION 0x05

#define INA226_REG_SETTINGS 0x00
// Регистр конфигурации INA226 (16 бит), используется для настройки режима измерений
typedef union
{
    uint16_t raw;
    struct
    {
        uint8_t mode_1 : 1; // Режим измерения
        uint8_t mode_2 : 1;
        uint8_t mode_3 : 1;
        uint8_t v_sh_ct_0 : 1; // Время преобразования шунтового напряжения
        uint8_t v_sh_ct_1 : 1;
        uint8_t v_sh_ct_2 : 1;
        uint8_t v_bus_ct_0 : 1; // Время преобразования шинного напряжения
        uint8_t v_bus_ct_1 : 1;
        uint8_t v_bus_ct_2 : 1;
        uint8_t avg_0 : 1; // Усреднение выборок
        uint8_t avg_1 : 1;
        uint8_t avg_2 : 1;
        uint8_t d_12 : 1; // Зарезервировано, не используется
        uint8_t d_13 : 1;
        uint8_t d_14 : 1;
        uint8_t rst : 1; // Сброс конфигурации при установке в 1
    };
} ina226_settings_register_t;

// Здесь хранятся последние показания датчика
struct battery_data
{
    uint16_t busVoltage; // mV
    uint16_t current;    // mA
    // uint16_t power;      // mW
} batteryData;

float currentLimitEnabled = 1; // Отключать моторы при высоком токе (BATTERY_DISARMED_CURRENT_THRESHOLD_MAX) и низком токе (BATTERY_DISARMED_CURRENT_THRESHOLD_MIN)

void setupBattery()
{
    print("Setup Battery\n");
    if (!resetINA226())
    {
        print("Сброс INA226 не выполнен\n");
        return;
    }
    if (!configureINA226())
    {
        print("Настройка INA226 не выполнена\n");
        return;
    }
    if (!calibrateINA226())
    {
        print("Калибровка INA226 не выполнена\n");
        return;
    }
}

// Сброс настроек INA226
bool resetINA226()
{
    ina226_settings_register_t resetSettings = {.rst = 1};
    Wire.beginTransmission(INA226_DEVICE_ADDRESS);
    Wire.write(INA226_REG_SETTINGS);
    Wire.write(resetSettings.raw >> 8);
    Wire.write(resetSettings.raw & 0xFF);
    if (Wire.endTransmission(true) != 0)
    {
        print("Ошибка при отправке команды на сброс настроек INA226\n");
        return false;
    }
    return true;
}

// Настройка INA226
bool configureINA226()
{
    uint8_t defaultSettingsBytes[2];
    if (!readRegister(&Wire, INA226_DEVICE_ADDRESS, INA226_REG_SETTINGS, defaultSettingsBytes, 2))
    {
        print("Ошибка при получении данных INA226_REG_SETTINGS\n");
        return false;
    }
    ina226_settings_register_t defaultSettings;
    defaultSettings.raw = (defaultSettingsBytes[0] << 8) | defaultSettingsBytes[1];

    // Изменить только то, что нам нужно, остальное оставить по умолчанию
    // Настройка датчика: Averages: 16, v bus conversion time: 2.116ms, v shunt conversion time: 2.116ms
    // 2.116 * 2 * 16 = 67.712 ms интервал обновления данных в датчике
    defaultSettings.mode_1 = 1;
    defaultSettings.mode_2 = 1;
    defaultSettings.mode_3 = 1;
    defaultSettings.v_sh_ct_0 = 1;
    defaultSettings.v_sh_ct_1 = 0;
    defaultSettings.v_sh_ct_2 = 1;
    defaultSettings.v_bus_ct_0 = 1;
    defaultSettings.v_bus_ct_1 = 0;
    defaultSettings.v_bus_ct_2 = 1;
    defaultSettings.avg_0 = 0;
    defaultSettings.avg_1 = 1;
    defaultSettings.avg_2 = 0;
    defaultSettings.rst = 1;
    Wire.beginTransmission(INA226_DEVICE_ADDRESS);
    Wire.write(INA226_REG_SETTINGS);
    Wire.write(defaultSettings.raw >> 8);
    Wire.write(defaultSettings.raw & 0xFF);
    if (Wire.endTransmission(true) != 0)
    {
        print("Ошибка при отправке команды на установку настроек INA226_REG_SETTINGS на INA226\n");
        return false;
    }
    return true;
}

// Калибровка INA226
bool calibrateINA226()
{
    Wire.beginTransmission(INA226_DEVICE_ADDRESS);
    Wire.write(INA226_REG_CALIBRATION);
    Wire.write((INA226_CURRENT_CALIBRATION >> 8) & 0xFF);
    Wire.write(INA226_CURRENT_CALIBRATION & 0xFF);
    if (Wire.endTransmission(true) != 0)
    {
        print("Ошибка при отправке команды на установку настроек INA226_REG_CALIBRATION на INA226\n");
        return false;
    }
    return true;
}

/*
 * Периодически считывает данные напряжения и тока с датчика INA226
 * с интервалом BATTERY_UPDATE_INTERVAL и сохраняет их в batteryData.
 */
void loopBattery()
{
    static double lastUpdateTime = 0.0;
    if (t - lastUpdateTime > BATTERY_UPDATE_INTERVAL)
    {
        update();
        // print("bus voltage: %d, current: %d, power: %d\n", batteryData.busVoltage, batteryData.current, batteryData.power);
        lastUpdateTime = t;
    }
    if (armed)
    {
        if (batteryData.busVoltage < BATTERY_DISARMED_VOLTAGE_THRESHOLD)
        {
            armed = false;
            print("Disarmed из-за низкого заряда аккумулятора: %d < %d\n", batteryData.busVoltage, BATTERY_DISARMED_VOLTAGE_THRESHOLD);
        }
        if (currentLimitEnabled)
        {
            if (batteryData.current > BATTERY_DISARMED_CURRENT_THRESHOLD_MAX)
            {
                armed = false;
                print("Disarmed из-за высокого тока: %d > %d\n", batteryData.current, BATTERY_DISARMED_CURRENT_THRESHOLD_MAX);
            }
            if (batteryData.current < BATTERY_DISARMED_CURRENT_THRESHOLD_MIN)
            {
                armed = false;
                print("Disarmed из-за низкого тока: %d < %d\n", batteryData.current, BATTERY_DISARMED_CURRENT_THRESHOLD_MIN);
            }
        }
    }
}

// Получение данных с датчика
static void update()
{
    uint8_t busVoltageRaw8[2], currentRaw8[2];

    if (readRegister(&Wire, INA226_DEVICE_ADDRESS, INA226_REG_BUS_VOLTAGE, busVoltageRaw8, 2))
    {
        const uint16_t busVoltageRaw16 = (busVoltageRaw8[0] << 8) | busVoltageRaw8[1];
        // Перевод значения напряжения: 1 LSB = 1.25 мВ
        batteryData.busVoltage = busVoltageRaw16 * 1.25;
    }
    else
    {
        print("Ошибка чтения данных напряжения\n");
    }

    if (readRegister(&Wire, INA226_DEVICE_ADDRESS, INA226_REG_CURRENT, currentRaw8, 2))
    {
        const uint16_t currentRaw16 = (currentRaw8[0] << 8) | currentRaw8[1];
        // Перевод значения тока: переводим из ампер в миллиамперы
        batteryData.current = currentRaw16 * INA226_CURRENT_LSB * 1000;
    }
    else
    {
        print("Ошибка чтения данных тока\n");
    }
}