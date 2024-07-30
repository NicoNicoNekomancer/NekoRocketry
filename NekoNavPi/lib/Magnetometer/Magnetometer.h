/*--------------------------------------------------------------------------------------------

has r/w bit. if 1, read, if 0, write
next bit is MS. If 0, dont increment, if 1, do. for multiread / write
next to the end of the first byte is the address
next byte is data that is read or written
https://www.st.com/resource/en/datasheet/lis3mdl.pdf

currently +- 8 gauss
full values are the following

FS      Scale     Register
+-4     6842      0 0
+-8     3421      0 1
+-12    2281      1 0
+-16    1711      1 1

--------------------------------------------------------------------------------------------*/

#ifndef Magnetometer_h
#define Magnetometer_h
#include <Arduino.h>
#include <SPI.h>

class Magnetometer {
public:
    Magnetometer(int csPin, int mosiPin, int misoPin, int sckPin, int clock);
    void begin();
    void readData(float &MagnetometerX, float &MagnetometerY, float &MagnetometerZ, float magOffset[3], float magSoftIron[3][3]);
    void SelfTest();
    void storeOffset(float gaussValueX, float gaussValueY, float gaussValueZ);

private: 
    const uint8_t CTRL_REG1 = 0x20;
    const uint8_t CTRL_REG2 = 0x21;
    const uint8_t CTRL_REG3 = 0x22;
    const uint8_t CTRL_REG4 = 0x23;
    const uint8_t CTRL_REG5 = 0x24;
    //using temp sensor, lpm, Fast_odr to bypass 80hz
    const uint8_t CTRL_REG1_Command = 0x82;
    //+- 16 gauss
    const uint8_t CTRL_REG2_Command = 0x60;
    //dont need to touch for now
    const uint8_t CTRL_REG3_Command = 0x00;
    //z axis high performance
    const uint8_t CTRL_REG4_Command = 0x00;
    //dont need to touch for now
    const uint8_t CTRL_REG5_Command = 0x00;

    const uint8_t DataStart = 0x28;

    const uint8_t Dummy = 0x00;

    int _csPin, _mosiPin, _misoPin, _sckPin;
    float scale = 0.0005844535;
    uint8_t OFFSET[6];
    uint8_t OFFSETSize = 6;
    uint8_t MagData[6];
    uint8_t SelfTestData[6];
    uint8_t MagDataSize = 6;
    uint8_t Status;

    SPISettings _spiSettings;

    void WriteReg(uint8_t Address, uint8_t Data);
    
    uint8_t* ReadReg(uint8_t addressStart, uint8_t Data[], uint8_t Size);
    uint8_t ReadStatus(uint8_t Data);
    int16_t twosComplement(uint8_t highByte, uint8_t lowByte);

    void init();
    float SelfTestX1, SelfTestY1, SelfTestZ1, SelfTestX2, SelfTestY2, SelfTestZ2, diffX, diffY, diffZ;
};

#endif
