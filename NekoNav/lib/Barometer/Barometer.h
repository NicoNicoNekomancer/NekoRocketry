#ifndef Barometer_h
#define Barometer_h

#include <Arduino.h>
#include <SPI.h>

class Barometer {
public:
    Barometer(int csPin, int mosiPin, int misoPin, int sckPin, int clock);
    void begin();
    void readData(float &temperature, float &pressure);

private:
    const uint8_t MS5611_CMD_READ_ADC = 0x00;
    const uint8_t MS5611_CMD_READ_PROM_FIRST = 0xA0;
    const uint8_t MS5611_CMD_READ_PROM_LAST = 0xAE;
    const uint8_t MS5611_CMD_RESET = 0x1E;
    const uint8_t MS5611_CMD_CONVERT_D1 = 0x48; // OSR 4096
    const uint8_t MS5611_CMD_CONVERT_D2 = 0x58; // OSR 4096

    int _csPin, _mosiPin, _misoPin, _sckPin;
    SPISettings _spiSettings;
    float _constants[8] = {1, 32768, 65536, 0.00390625, 0.0078125, 256, 0.00000011920928955078125, 1};
    uint32_t readAdc(uint8_t command);
    float dT, TEMP, T2, t, P, OFF, SENS, OFF2, SENS2;
    


    void reset();
    void readProm();
    uint16_t getProm(uint8_t reg);
    void calculateCompensation(uint32_t D1, uint32_t D2);
    
};

#endif
