#ifndef W25Q_h
#define W25Q_h

#include <Arduino.h>
#include <SPI.h>

class W25Q {
public:
    W25Q(int csPin, int mosiPin, int misoPin, int sckPin, int clock);
    void begin();
    

private: 

    const uint8_t Dummy = 0x00;
    const WriteEnable = 0x06;
    const WriteDisable = 0x04;
    const ReadData = 0x03;
    const WritePage = 0x02;




    int _csPin, _mosiPin, _misoPin, _sckPin;
    SPISettings _spiSettings;


    void WriteFlash(uint8_t Address, uint8_t Data);
    void ReadFlash(uint8_t Address, uint8_t Data);

};

#endif
