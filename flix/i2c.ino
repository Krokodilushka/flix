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