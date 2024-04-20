#include "W25Q.h"

/*
Notes on all of the data stuff
Data is to be written from 1 byte to 256 bytes. This is one page.
to write over this data, you must erase it. This is done by, at minimum, a sectior, which is 4Kb or 16 pages
Therefor, all programmed data will take up the first sector or 16 pages. This is a waste of data but this is how it goes. 

*/
W25Q::W25Q(int csPin, int mosiPin, int misoPin, int sckPin, int clock) {
    _csPin = csPin;
    _mosiPin = mosiPin;
    _misoPin = misoPin;
    _sckPin = sckPin;
    _spiSettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
}

void W25Q::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.setRX(_misoPin);
    SPI.setTX(_mosiPin);
    SPI.setSCK(_sckPin);
    SPI.begin();
    delay(100);
    init();
}

void W25Q::WriteFlash(uint8_t address, uint8_t data[]) {
    /*
    enable write, send address, then the page. can send from one byte to 256 bytes (one page). If 256 is to be sent, the last address byte houyld be set to 0
    each transfer writes the next bit of data so n+1
    end with cs set to high
    WriteDisable is automatically set at the end of the page. 
    can only erase in sectors, which is minimum 4kb aka 16 pages
    address is A23-A0 or 2595 - 160
    MAKE SURE NOT TO SEND MORE THAN 256 BYTES  
    */
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(WriteEnable);
    digitalWrite(_csPin, HIGH);

    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(WritePage);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8) & 0xFF);
    SPI.transfer(address & 0xFF);
    for(int i = 0; i < sizeof(data); i++){
      SPI.transfer(data(i));
    }
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();







    
}

void W25Q::ReadFlash(uint8_t address) {
    //Send the command and then the address. Each read after the address sends the next one, so n+1 type read. completed when cs is high. 
    //This one is for exporting data
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(ReadData);
    SPI.transfer(address);
    digitalWrite(_csPin, HIGH);









    SPI.endTransaction();
}