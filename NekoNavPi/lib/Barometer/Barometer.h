
/*
this chip also kinda sucks
it has a built in delay for the calculation that you cant get around. 
its based on what resoltuion / osr you use, BUT, it does lets you do other stuff while it calculates
so this means that you need to first call the d1 calc, then do the rest of the sensors, then do the d2 calc,
then do everything else, and then collect the data

https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036
  OSR:256 OSR:512 OSR:1024  OSR:2048  OSR:4096
D1  0x40    0x42    0x44      0x46      0x48
D2  0x50    0x52    0x54      0x56      0x58

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
    const uint8_t MS5611_CMD_CONVERT_D1 = 0x42; // OSR
    const uint8_t MS5611_CMD_CONVERT_D2 = 0x52; // OSR

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
