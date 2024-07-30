
/*
this chip also kinda sucks
it has a built in delay for the calculation that you cant get around. 
its based on what resoltuion / osr you use, BUT, it does lets you do other stuff while it calculates
so this means that you need to first call the d1 calc, then do the rest of the sensors, then do the d2 calc,
then do everything else, and then collect the data
*/
#ifndef Barometer_h
#define Barometer_h

#include <Arduino.h>
#include <SPI.h>

class Barometer {
public:
    Barometer(int csPin, int mosiPin, int misoPin, int sckPin, int clock);
    void begin();
    //send these twice
    void startADC();
    void readData(float &temperature, float &pressure);
    //send these twice
    
private:
    const uint8_t MS5611_CMD_READ_ADC = 0x00;
    const uint8_t MS5611_CMD_READ_PROM_FIRST = 0xA0;
    const uint8_t MS5611_CMD_READ_PROM_LAST = 0xAE;
    const uint8_t MS5611_CMD_RESET = 0x1E;
    const uint8_t MS5611_CMD_CONVERT_D1 = 0x44; // OSR 1024
    const uint8_t MS5611_CMD_CONVERT_D2 = 0x54; // OSR 1024

    int _csPin, _mosiPin, _misoPin, _sckPin;
    SPISettings _spiSettings;
    float _constants[8] = {1, 32768, 65536, 0.00390625, 0.0078125, 256, 0.00000011920928955078125, 1};
    uint32_t readAdc();
    float dT, TEMP, T2, t, P, OFF, SENS, OFF2, SENS2, D1, D2, delay1, delay2;
    

    bool C1 = false;
    bool C2 = false;
    void reset();
    void readProm();
    uint16_t getProm(uint8_t reg);
    void calculateCompensation(uint32_t D1, uint32_t D2);
    
};

#endif
