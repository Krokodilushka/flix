/*
 * Управление дроном через Bluetooth-геймпад Flydigi Vader 3 pro с использованием библиотеки NimBLE.
 * Код обрабатывает подключение к геймпаду, получение данных о состоянии кнопок и стиков, а также преобразование их в команды управления дроном.
 * Поддерживается автоматическое сканирование устройств и failsafe при потере связи.
 *
 * В flix.ino нужно установить GAMEPAD_ENABLED = 1.
 * Включить дрон, включить геймпад в режиме bluetooth (переключатель на обратной стороне геймпада, среднее положение).
 * Во время ожидания подключения лампочка на геймпаде должна мигать синим цветом, при активном соединении будет постоянно гореть синим.
 * Управление:
 * B — Отключение моторов (disarmed). Сбрасывает тягу на ноль.
 * A — Включение моторов (armed). Сбрасывает тягу на ноль и разрешает работу моторов.
 * LT/RT - Убавляет/прибавляет тягу. Моторы не начнут работать если сначала не нажать A (armed).
 * LB/RB - Наклоны влево/вправо
 * Левый стик - Наклоны вперед/назад, поворты влево/вправо
 * START - Перезагрузка ESP32
 * C - калибровка гироскопа
 */

#if GAMEPAD_ENABLED

extern int rollChannel, pitchChannel, throttleChannel, yawChannel, armedChannel, modeChannel; // Нужны для управления дроном
extern double controlsTime;                                                                   // Нужен для работы failsafe

#define THROTTLE_CHANGE_INTERVAL 0.01f // С каким интервалом регулировать тягу (10 мс)
#define GAMEPAD_STICK_L_DEADZONE 0.05f // Не реагировать на небольшие движения левого стика
#define RSSI_INTERVAL 1.0

NimBLEUUID serviceUUID("1812");        // Служба HID, которая должна быть на геймпаде
NimBLEUUID characteristicUUID("2A4D"); // Характеристика службы HID, которая должна быть на геймпаде

/*
 * Если адрес указан, подключается только к геймпаду с этим адресом.
 * Если адрес не указан, выполняется сканирование и выбирается устройство с нужным сервисом, характеристикой и наилучшим сигналом (RSSI).
 * Адрес геймпада можно будет посмотреть в логах.
 */
// NimBLEAddress gamepadAddress = NimBLEAddress("a4:c1:38:30:a7:33"); // Адрес геймпада для прямого подключения только к нему
NimBLEAddress gamepadAddress = NimBLEAddress(); // Пустой адрес для автоматического выбора геймпада

NimBLEClient *pClient = nullptr; // После вызова setupGamepad здесь будет bluetooth клиент
// Состояния подключения геймпада
enum class ConnectionState
{
    DISCONNECTED, // Геймпад не подключен
    CONNECTING,   // Идёт попытка подключения к геймпаду
    CONNECTED,    // Геймпад подключён, но не подписан на уведомления о состоянии кнопок
    SUBSCRIBED    // Подписка на уведомления о состоянии кнопок выполнена
};
ConnectionState connectionState = ConnectionState::DISCONNECTED; // Текущее состояние подключения
double throttleHandleTime = 0.0;
double loopGamepadUpdateRssTime = 0.0;
uint8_t gamepadRssi = 0;

// Ограничения для стиков и триггеров обеспечивают плавное управление. Могут быть настроены через QGroundControl.
struct gamepad_limits
{
    float throttle;
    float pitch;
    float roll;
    float yaw;
} gamepadLimits = {0.002f, 0.4f, 0.4f, 0.5f};

// Текущее состояние всех кнопок и стиков геймпада, обновляется при получении данных по BLE
struct
{
    int8_t stick_l_x;    // Левый стик X (-128..127)
    int8_t stick_l_y;    // Левый стик Y (-128..127)
    bool stick_l_button; // Нажатие на левый стик
    int8_t stick_r_x;    // Правый стик X (-128..127)
    int8_t stick_r_y;    // Правый стик Y (-128..127)
    bool stick_r_button; // Нажатие на правый стик
    uint8_t lt;          // Левый триггер (0..255)
    uint8_t rt;          // Правый триггер (0..255)
    bool lb;             // Левый бампер
    bool rb;             // Правый бампер
    bool a;              // Кнопка A
    bool x;              // Кнопка X
    bool y;              // Кнопка Y
    bool b;              // Кнопка B
    bool select;         // Кнопка Select
    bool start;          // Кнопка Start
    bool cd;             // Крестовина вниз
    bool cdl;            // Крестовина вниз-влево
    bool cl;             // Крестовина влево
    bool clu;            // Крестовина влево-вверх
    bool cu;             // Крестовина вверх
    bool cur;            // Крестовина вверх-вправо
    bool cr;             // Крестовина вправо
    bool crd;            // Крестовина вправо-вниз
    bool c;              // Кнопка C
    bool z;              // Кнопка Z
    bool o;              // Кнопка слева от home
    bool home;           // Кнопка home
    bool m1;             // Кнопка на обратной стороне
    bool m3;             // Кнопка на обратной стороне
    bool m4;             // Кнопка на обратной стороне
    bool m2;             // Кнопка на обратной стороне
} gamepadState;

// Callback для подключения/отключения
class ClientCallback : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient *client) override
    {
        connectionState = ConnectionState::CONNECTED;
        NimBLEConnInfo config = client->getConnInfo();
        print("Геймпад подключён. Разъединение при таймауте: %d мс, макс байт в пакете данных: %d\n", config.getConnTimeout() * 10, config.getMTU());
    }

    void onConnectFail(NimBLEClient *client, int reason) override
    {
        connectionState = ConnectionState::DISCONNECTED;
        print("Ошибка подключения геймпада\n");
    }

    // Срабатывает примерно через 3 секунды после отключения геймпада.
    void onDisconnect(NimBLEClient *pClient, int reason) override
    {
        connectionState = ConnectionState::DISCONNECTED;
        print("Геймпад отключён\n");
        gamepadState = {};  // Состояние нажатых кнопок сбрасывается
        controlsTime = 0.0; // Для быстрого срабатывания failsafe
    }
};

void setupGamepad()
{
    NimBLEDevice::init("Flix drone");
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallback());
    // client->setConnectionParams(40, 48, 0, 100); // Установка 1000 мс (100 * 10) таймаута (не работает)
}

void loopGamepad()
{
    if (!pClient)
    {
        print("BLE клиент не найден (не был вызван setupGamepad())\n");
        return;
    }
    switch (connectionState)
    {
    case ConnectionState::CONNECTED:
        while (!subscribe())
        {
            print("Ошибка при подписке на данные о нажатии кнопок геймпада. Повтор через 1 сек\n");
            delay(1000);
        }
        connectionState = ConnectionState::SUBSCRIBED;
        print("Успешно подписался на уведомления о нажатии кнопок геймпада\n");
        break;
    case ConnectionState::SUBSCRIBED:
        controlsTime = t;
        if (t - loopGamepadUpdateRssTime > RSSI_INTERVAL)
        {
            gamepadRssi = pClient->getRssi();
            loopGamepadUpdateRssTime = t;
        }
        handleThrottle();
        break;
    case ConnectionState::DISCONNECTED:
        if (gamepadAddress.isNull())
        {
            print("Запуск сканирования BLE геймпадов\n");
            scan();
        }
        else
        {
            print("Подключение к BLE геймпаду\n");
            connectionState = ConnectionState::CONNECTING;
            bool deleteAttributes = true; // При false геймпад не всегда определяет что к нему подключились и через какое то время выключается
            bool asyncConnect = true;     // Асинхронное подключение чтобы не держать поток. В случае успеха сработает ClientCallback::onConnect, в случае ошибки ClientCallback::onConnectFail
            bool exchangeMTU = true;      // Узнать у геймада максимальный рамер пакета
            pClient->connect(gamepadAddress, deleteAttributes, asyncConnect, exchangeMTU);
        }
        break;
    default:
        break;
    }
}

/*
 * Функция нужна для плавного прибавления/убавления тяги пока нажата соответствующая кнопка.
 * Если нажать на максимум LT/RT, то событие о переключении триггера на 1 придет только один раз.
 * Второе событие может прийти через несколько секунд, когда триггер будет приотпущен на любое другое значение.
 * Все это время нужно считать что триггер нажат и нужно прибавлять или убавлять тягу.
 */
void handleThrottle()
{
    if (t - throttleHandleTime > THROTTLE_CHANGE_INTERVAL)
    {
        const int16_t throttle = gamepadState.rt - gamepadState.lt;                               // Если конопки тяги прижаты на равное значение, то уровень тяги не изменится
        float value = mapf(throttle, -255, 255, -gamepadLimits.throttle, gamepadLimits.throttle); // Ограничить влияние кнопок для плавности
        controls[throttleChannel] = constrain(controls[throttleChannel] + value, 0.f, 1.f);       // Прибавить к текущему уровню тяги
        throttleHandleTime = t;
    }
}

// Сканирование BLE устройств, выбор подходящего и сохрание его адреса в переменную чтобы позже подключиться
void scan()
{
    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true); // Без активного сканирования не работает isAdvertisingService
    NimBLEScanResults results = pScan->getResults(1000);
    struct
    {
        bool isFound;
        int rssi;
        NimBLEAddress address;
    } deviceToConnect = {false, -1000, NimBLEAddress()}; // Для сохранения подходящего устройства для подключения
    for (int i = 0; i < results.getCount(); i++)
    {
        const NimBLEAdvertisedDevice *device = results.getDevice(i);
        const bool hasService = device->isAdvertisingService(serviceUUID);
        const int rssi = device->getRSSI();
        const NimBLEAddress address = device->getAddress();
        print(
            "Устройство: %s, RSSI: %d, %s, имя: %s\n",
            address.toString().c_str(),
            rssi,
            hasService ? "нужный сервис поддерживается" : "нужный сервис не поддерживается",
            device->getName().c_str());
        // Сохранить для подключения, если найденное устройство (геймпад) лучше по качеству связи (ближе к дрону)
        if (hasService && rssi > deviceToConnect.rssi)
        {
            deviceToConnect = {.isFound = true, .rssi = rssi, .address = address};
        }
    }
    if (deviceToConnect.isFound)
    {
        print("Выбрано устройство для подключения: %s, RSSI: %d\n", deviceToConnect.address.toString().c_str(), deviceToConnect.rssi);
        gamepadAddress = deviceToConnect.address;
    }
    else
    {
        print("Не найдено подходящее устройство для подключения\n");
    }
}

/*
 * Подписывается на уведомления о состоянии кнопок и стиков геймпада через.
 * Возвращает true при успешной подписке, false при ошибке.
 */
bool subscribe()
{
    print("Подписка на уведомления о нажатии кнопок геймпада\n");
    NimBLERemoteService *pService = pClient->getService(serviceUUID); // Попытка получить нужный сервис
    if (!pService)
    {
        print("Служба HID не найдена!\n");
        return false;
    }

    NimBLERemoteCharacteristic *pChar = pService->getCharacteristic(characteristicUUID); // Попытка получить нужную характеристику сервиса
    if (pChar == nullptr)
    {
        print("Характеристика HID %s не найдена!\n", characteristicUUID.toString().c_str());
        return false;
    }

    // Проверяем есть ли уведомления у нужной характеристики
    if (pChar->canNotify())
    {
        if (!pChar->subscribe(true, &onCharacteristicCallback))
        {
            print("Ошибка подписки на уведомления!\n");
            return false;
        }
    }
    else
    {
        print("У характеристики %s нет уведомлений\n", characteristicUUID.toString().c_str());
        return false;
    }

    return true;
}

void onCharacteristicCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData, size_t length, bool isNotify)
{
    // Может пригодиться для определения нажатых кнопок для других геймпадов
    // Serial.print("Данные: ");
    // for (int i = 0; i < length; i++)
    // {
    //     Serial.printf("[%d:", i);
    //     for (int8_t b = 7; b >= 0; b--)
    //     {
    //         Serial.printf("%d", (pData[i] >> b) & 1);
    //     }
    //     Serial.print("] ");
    // }
    // Serial.print(" ");

    updateControlsState(pData, length); // Сохранить состояние кнопок в переменную
    // Serial.printf("STICK_L_X: %d, STICK_L_Y: %d, STICK_L_BUTTON: %d, STICK_R_X: %d, STICK_R_Y: %d, STICK_R_BUTTON: %d, LT: %d, RT: %d, LB: %d, RB: %d, A: %d, X: %d, Y: %d, B: %d, select: %d, start: %d, CD: %d, CDL: %d, CL: %d, CLU: %d, CU: %d, CUR: %d, CR: %d, CRD: %d, C: %d, Z: %d, O: %d, HOME: %d, M1: %d, M3: %d, M4: %d, M2: %d\n/////////\n",
    //               gamepadState.stick_l_x,
    //               gamepadState.stick_l_y,
    //               gamepadState.stick_l_button,
    //               gamepadState.stick_r_x,
    //               gamepadState.stick_r_y,
    //               gamepadState.stick_r_button,
    //               gamepadState.lt,
    //               gamepadState.rt,
    //               gamepadState.lb,
    //               gamepadState.rb,
    //               gamepadState.a,
    //               gamepadState.x,
    //               gamepadState.y,
    //               gamepadState.b,
    //               gamepadState.start,
    //               gamepadState.select,
    //               gamepadState.cd,
    //               gamepadState.cdl,
    //               gamepadState.cl,
    //               gamepadState.clu,
    //               gamepadState.cu,
    //               gamepadState.cur,
    //               gamepadState.cr,
    //               gamepadState.crd,
    //               gamepadState.c,
    //               gamepadState.z,
    //               gamepadState.o,
    //               gamepadState.home,
    //               gamepadState.m1,
    //               gamepadState.m3,
    //               gamepadState.m4,
    //               gamepadState.m2);
    // return;

    // Кнопка B моментально отключает моторы (disarmed)
    // Если нажаты кнопки A и B одновременно, то сработает только B (disarmed) и моторы не включатся
    if (gamepadState.b)
    {
        // Тягу на 0
        controls[throttleChannel] = 0.f;
        if (controls[armedChannel] != 0.f)
        {
            controls[armedChannel] = 0.f;
            print("Disarmed\n");
        }
    }
    // Кнопка A разрешает моторам работать (armed)
    else if (gamepadState.a)
    {
        if (controls[armedChannel] != 1.f)
        {
            // На всякий случай тягу на 0
            controls[throttleChannel] = 0.f;
            controls[armedChannel] = 1.f;
            print("Armed\n");
        }
    }

    // Верхние кнопки LB/RB управляют наклоном влево/впараво (крен/roll)
    controls[rollChannel] = mapf(gamepadState.rb - gamepadState.lb, -1, 1, -gamepadLimits.roll, gamepadLimits.roll);

    // Левый стик вперед/назад управляет наклоном вперед/назад (тангаж/pitch)
    controls[pitchChannel] = mapf(gamepadState.stick_l_y, -128, 127, gamepadLimits.pitch, -gamepadLimits.pitch);
    // Не реагировать, если движение стика меньше чем deadzone
    if (controls[pitchChannel] > -GAMEPAD_STICK_L_DEADZONE && controls[pitchChannel] < GAMEPAD_STICK_L_DEADZONE)
    {
        controls[pitchChannel] = 0;
    }

    // Левый стик вправо/влево управляет поворотом (рысканье/yaw)
    controls[yawChannel] = mapf(gamepadState.stick_l_x, -128, 127, -gamepadLimits.yaw, gamepadLimits.yaw);
    // Не реагировать, если движение стика меньше чем deadzone
    if (controls[yawChannel] > -GAMEPAD_STICK_L_DEADZONE && controls[yawChannel] < GAMEPAD_STICK_L_DEADZONE)
    {
        controls[yawChannel] = 0;
    }

    // По нажатию C запустить калибровку гироскопа
    if (gamepadState.c)
    {
        calibrateGyroOnce();
    }

    // По нажатию START перезагрузить ESP32
    if (gamepadState.start)
    {
        ESP.restart();
    }
}

/*
 * Преобразует данные, полученные по Bluetooth, в состояние кнопок и стиков геймпада.
 * Сохранённое состояние можно использовать в loopGamepad().
 */
void updateControlsState(uint8_t *data, size_t length)
{
    if (length < 14)
        return;

    gamepadState.stick_l_x = (int8_t)data[0];
    gamepadState.stick_l_y = (int8_t)data[1];
    gamepadState.stick_l_button = (data[9] & 1 << 6) != 0;
    gamepadState.stick_r_x = (int8_t)data[2];
    gamepadState.stick_r_y = (int8_t)data[3];
    gamepadState.stick_r_button = (data[9] & 1 << 7) != 0;
    gamepadState.lt = data[12];
    gamepadState.rt = data[13];
    gamepadState.lb = (data[9] & 1 << 0) != 0;
    gamepadState.rb = (data[9] & 1 << 1) != 0;
    gamepadState.a = (data[8] & 1 << 4) != 0;
    gamepadState.b = (data[8] & 1 << 5) != 0;
    gamepadState.x = (data[8] & 1 << 6) != 0;
    gamepadState.y = (data[8] & 1 << 7) != 0;
    gamepadState.select = (data[9] & 1 << 5) != 0;
    gamepadState.start = (data[9] & 1 << 4) != 0;
    gamepadState.cd = (data[8] & 0xFF) == 0b00000101;
    gamepadState.cdl = (data[8] & 0xFF) == 0b00000110;
    gamepadState.cl = (data[8] & 0xFF) == 0b00000111;
    gamepadState.clu = (data[8] & 0xFF) == 0b00001000;
    gamepadState.cu = (data[8] & 0xFF) == 0b00000001;
    gamepadState.cur = (data[8] & 0xFF) == 0b00000010;
    gamepadState.cr = (data[8] & 0xFF) == 0b00000011;
    gamepadState.crd = (data[8] & 0xFF) == 0b00000100;
    gamepadState.c = (data[10] & 1 << 0) != 0;
    gamepadState.z = (data[10] & 1 << 1) != 0;
    gamepadState.o = (data[10] & 1 << 7) != 0;
    gamepadState.home = (data[11] & 1 << 7) != 0;
    gamepadState.m1 = (data[10] & 1 << 3) != 0;
    gamepadState.m3 = (data[10] & 1 << 5) != 0;
    gamepadState.m4 = (data[10] & 1 << 4) != 0;
    gamepadState.m2 = (data[10] & 1 << 2) != 0;
}

#endif