#ifndef Magnetometer_h
#define Magnetometer_h
//has r/w bit. if 1, read, if 0, write
//next bit is MS. If 0, dont increment, if 1, do. for multiread / write
//next to the end of the first byte is the address
//next byte is data that is read or written
#include <Arduino.h>
#include <SPI.h>

class Magnetometer {
public:
    Magnetometer(int csPin, int mosiPin, int misoPin, int sckPin, int clock);
    void begin();
    void readData(float &MagnetometerX, float &MagnetometerY, float &MagnetometerZ);

private: 
    //reading these values so first bit 1
    const uint8_t CTRL_REG1 = 0x20;
    const uint8_t CTRL_REG2 = 0x21;
    const uint8_t CTRL_REG3 = 0x22;
    const uint8_t CTRL_REG4 = 0x23;
    const uint8_t CTRL_REG5 = 0x24;
    //using temp sensor, high performance, Fast_odr to bypass 80hz
    const uint8_t CTRL_REG1_Command = 0xC2;
    //+- 8 gauss
    const uint8_t CTRL_REG2_Command = 0x20;
    //dont need to touch for now
    const uint8_t CTRL_REG3_Command = 0x00;
    //z axis high performance
    const uint8_t CTRL_REG4_Command = 0x08;
    //dont need to touch for now
    const uint8_t CTRL_REG5_Command = 0x00;

    const uint8_t DataStart = 0x28;

    const uint8_t Dummy = 0x00;

    int _csPin, _mosiPin, _misoPin, _sckPin;
    int scale = 3421;
    uint8_t OFFSET[6];
    uint8_t OFFSETSize = 6;
    uint8_t MagData[6];
    uint8_t MagDataSize = 6;

    SPISettings _spiSettings;

    void WriteReg(uint8_t Address, uint8_t Data);
    uint8_t* ReadReg(uint8_t addressStart, uint8_t Data[], uint8_t Size);
    int16_t twosComplement(uint8_t highByte, uint8_t lowByte);
    void init();

};

#endif
