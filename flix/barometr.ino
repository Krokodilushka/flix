/*
 * Получает данные с барометра BMP580 с интервалом BAROMETER_UPDATE_INTERVAL.
 * Высота над уровнем моря отправляется в QGroundControl (в файле mavlink.ino). Для правильного расчета высоты нужно изменить значение переменной seaLevelPa.
 * Высоту над уровнем моря можно вывести в виджет.
 */
#if BAROMETER_ENABLED
    // Адреса регистров датчика
    #define BMP580_DEVICE_ADDRESS 0x47
    #define BMP580_REG_COMMAND 0x7E
    #define BMP580_COMMAND_SOFT_RESET 0xB6
    #define BMP580_REG_CHIP_ID 0x01
    #define BMP580_REG_TEMPERATURE_XLSB 0x1D

    // Далее структуры для данных из регистров BMP580
    #define BMP580_REG_STATUS 0x28
    typedef struct
    {
        uint8_t reserved0 : 1;
        uint8_t status_nvm_rdy : 1;
        uint8_t status_nvm_err : 1;
        uint8_t status_nvm_cmd_err : 1;
        uint8_t reserved_4 : 1;
        uint8_t reserved_6_5 : 2;
        uint8_t reserved7 : 1;
    } bmp580_status_register_t;

    #define BMP580_REG_INTERRUPT_STATUS 0x27
    typedef struct
    {
        uint8_t drdy_data_reg : 1;
        uint8_t fifo_full : 1;
        uint8_t fifo_ths : 1;
        uint8_t oor_p : 1;
        uint8_t por : 1;
        uint8_t reserved_7_5 : 3;
    } bmp580_interrupt_status_register_t;

    #define BMP580_REG_ODR 0x37
    typedef struct
    {
        uint8_t pwr_mode : 2;
        uint8_t odr : 5;
        uint8_t deep_dis : 1;
    } bmp580_odr_register_t;

    #define BMP580_REG_OSR 0x36
    typedef struct
    {
        uint8_t osr_t : 3;
        uint8_t osr_p : 3;
        uint8_t press_en : 1;
        uint8_t reserved_7 : 1;
    } bmp580_osr_register_t;

    #define BMP580_REG_EFFECTIVE_OSR 0x38
    typedef struct
    {
        uint8_t odr_is_valid : 1;
        uint8_t reserved_6 : 1;
        uint8_t osr_p_eff : 3;
        uint8_t osr_t_eff : 3;
    } bmp580_effective_osr_register_t;

    // Здесь хранятся последние показания датчика
    struct
    {
        float pa;          // Давление в Па
        float temperature; // Температура в градусах цельсия
    } barometrData;

    /*
    * Для расчета текущей высоты над уровнем моря нужно установить текущее давление на уровне моря в вашем регионе.
    * Давление в мм рт. ст. можно посмотреть на сайте https://primpogoda.ru/weather/moskva в строке "Атм. давление на уровне моря". Значение нужно умножить на 133.322.
    * Пример: 747 * 133.322 = 99591.534
    * Эту переменную можно установить через QGroundControl
    */
    float seaLevelPa = 99324.89;

    void setupBarometer()
    {
        print("Setup Barometer\n");
        if (!resetBMP580())
        {
            print("Сброс настроек BMP580 не выполнен\n");
            return;
        }
        delay(10); // Пауза на 10мс. По документации сброс занимает 2мс, но у меня на практике оказалось что нужно больше времени.
        if (!checksBMP580())
        {
            print("Не пройдены проверки датчика BMP580\n");
            return;
        }
        if (!configureBMP580())
        {
            print("Настройка BMP580 не выполнена\n");
            return;
        }
        return;
    }

    // Сброс настроек BMP580
    bool resetBMP580()
    {
        Wire.beginTransmission(BMP580_DEVICE_ADDRESS);
        Wire.write(BMP580_REG_COMMAND);
        Wire.write(BMP580_COMMAND_SOFT_RESET);
        if (Wire.endTransmission(true) != 0)
        {
            print("Ошибка при отправке команды на сброс настроек BMP580\n");
            return false;
        }
        return true;
    }

    // Проверка датчика. В документации рекомендуют после запуска сделать проверку трех регистров.
    bool checksBMP580()
    {
        // Проверка что CHIP_ID !=0
        uint8_t dataChipId[1];
        if (readRegister(&Wire, BMP580_DEVICE_ADDRESS, BMP580_REG_CHIP_ID, dataChipId, 1))
        {
            if (dataChipId[0] == 0)
            {
                print("Не пройдена проверка по CHIP_ID\n");
                return false;
            }
        }
        else
        {
            print("Ошибка при получении данных BMP580_REG_CHIP_ID\n");
            return false;
        }

        // Проверка что STATUS.status_nvm_rdy == 1 && STATUS.status_nvm_err == 0
        bmp580_status_register_t statusReg;
        if (readRegister(&Wire, BMP580_DEVICE_ADDRESS, BMP580_REG_STATUS, (uint8_t *)&statusReg, 1))
        {
            if (!(statusReg.status_nvm_rdy == 1 && statusReg.status_nvm_err == 0))
            {
                print("Не пройдена проверка status_reg.status_nvm_rdy: %d != 1 или status_reg.status_nvm_err: %d !=0\n", statusReg.status_nvm_rdy, statusReg.status_nvm_err);
                return false;
            }
        }
        else
        {
            print("Ошибка при получении данных BMP580_REG_STATUS\n");
            return false;
        }

        // Проверка INT_STATUS.por == 1
        bmp580_interrupt_status_register_t interrupt_status_reg;
        if (readRegister(&Wire, BMP580_DEVICE_ADDRESS, BMP580_REG_INTERRUPT_STATUS, (uint8_t *)&interrupt_status_reg, 1))
        {
            if (interrupt_status_reg.por != 1)
            {
                print("Не пройдена проверка interrupt_status_reg.por: %d != 1\n", interrupt_status_reg.por);
                return false;
            }
        }
        else
        {
            print("Ошибка при получении данных BMP580_REG_INTERRUPT_STATUS\n");
            return false;
        }
        return true;
    }

    // Настройка BMP580
    bool configureBMP580()
    {
        // Настройка Over-sampling rate. При настройке osr_p = 128x, osr_t = 8x максимальная скорость обновления данных будет 12 Hz (если датчик в CONTINUOUS режиме)
        bmp580_osr_register_t osrReg;
        osrReg.press_en = 0b1; // Включает измерение давления (в противном случае будет измеряться только температура)
        osrReg.osr_p = 0b111;  // Давление: oversampling rate = 128x
        osrReg.osr_t = 0b011;  // Температура: oversampling rate = 8x
        Wire.beginTransmission(BMP580_DEVICE_ADDRESS);
        Wire.write(BMP580_REG_OSR);
        Wire.write((osrReg.reserved_7 << 7) | (osrReg.press_en << 6) | (osrReg.osr_p << 3) | osrReg.osr_t);
        if (Wire.endTransmission(true) != 0)
        {
            print("Ошибка при отправке команды на на установку настроек BMP580_REG_OSR на BMP580\n");
            return false;
        }

        // Настройка Output data rate
        bmp580_odr_register_t odrReg;
        odrReg.deep_dis = 0x1;  // Отключение DEEP STANDBY
        odrReg.pwr_mode = 0x01; // Нормальный режим
        odrReg.odr = 0x17;      // 10.000 Hz
        Wire.beginTransmission(BMP580_DEVICE_ADDRESS);
        Wire.write(BMP580_REG_ODR);
        Wire.write((odrReg.deep_dis << 7) | (odrReg.odr << 2) | odrReg.pwr_mode);
        if (Wire.endTransmission(true) != 0)
        {
            print("Ошибка при отправке команды на на установку настроек BMP580_REG_ODR на BMP580\n");
            return false;
        }

        // Проверка на то, что используется допустимая комбинация настроек Over-sampling rate и Output data rate
        bmp580_effective_osr_register_t effectiveOsrReg;
        if (!readRegister(&Wire, BMP580_DEVICE_ADDRESS, BMP580_REG_EFFECTIVE_OSR, (uint8_t *)&effectiveOsrReg, 1))
        {
            print("Ошибка при получении данных BMP580_REG_EFFECTIVE_OSR\n");
            return false;
        }
        // print("BMP580_REG_EFFECTIVE_OSR: odr_is_valid=%X, reserved_6=%X, osr_p_eff=%X, osr_t_eff=%X\n", effectiveOsrReg.odr_is_valid, effectiveOsrReg.reserved_6, effectiveOsrReg.osr_p_eff, effectiveOsrReg.osr_t_eff);
        if (effectiveOsrReg.odr_is_valid != 1)
        {
            print("Комбинация настроек Over-sampling rate и Output data rate не валидна\n");
            return false;
        }
        return true;
    }

    /*
    * Периодически считывает данные давления и температуры с датчика BMP580
    * с интервалом BAROMETER_UPDATE_INTERVAL и сохраняет их в barometrData.
    */
    void loopBarometer()
    {
        static double lastUpdateTime = 0.0;
        if (t - lastUpdateTime >= BAROMETER_UPDATE_INTERVAL)
        {
            uint8_t data[6];
            // Датчик позволяет за один раз считать данные о давлении и тепературе (burst read). 3 байта для давления и 3 для температуры.
            if (readRegister(&Wire, BMP580_DEVICE_ADDRESS, BMP580_REG_TEMPERATURE_XLSB, data, 6))
            {
                uint32_t rawTemperature = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
                barometrData.temperature = rawTemperatureToCelsius(rawTemperature); // Первые 3 байта переводятся в 32 битное число и конвертируетсяв градусы цельсия
                uint32_t rawPressure = ((uint32_t)data[5] << 16) | ((uint32_t)data[4] << 8) | data[3];
                barometrData.pa = rawPressureToPa(rawPressure);
            }
            else
            {
                print("Ошибка при чтении данных давления и температуры\n");
                return;
            }
            lastUpdateTime = t;
        }
    }

    // Переводит давление в метры над уровнем моря
    // Для корректного результата нужно знать текущее давление на уровне моря в вашем регионе.
    // Подробнее в комметарии к переменной seaLevelPa
    float paToAltitude(float pressurePa)
    {
        return 44330.0f * (1.0f - powf(pressurePa / seaLevelPa, 0.1903));
    }

    // Переводит из данных регистра датчика в ПА
    static float rawPressureToPa(uint32_t rawPressure)
    {
        return (float)(rawPressure >> 6);
    }

    // Переводит из данных регистра датчика в градусы цельсия
    static float rawTemperatureToCelsius(uint32_t rawTemperature)
    {
        return (float)(rawTemperature >> 16);
    }
#endif