#include "Magnetometer.h"
//has r/w bit. if 1, read, if 0, write
//next bit is MS. If 0, dont increment, if 1, do. for multiread / write
//next to the end of the first byte is the address
//next byte is data that is read or written
//some notes. for whatever reason it looks like the auto increment just doesnt work and wont get to temp. idk why

Magnetometer::Magnetometer(int csPin, int mosiPin, int misoPin, int sckPin, int clock) {
    _csPin = csPin;
    _mosiPin = mosiPin;
    _misoPin = misoPin;
    _sckPin = sckPin;
    _spiSettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
}

void Magnetometer::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin(_sckPin, _misoPin, _mosiPin, _csPin);
    delay(100);
    init();
    
}


void Magnetometer::WriteReg(uint8_t address, uint8_t command) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(address);
    SPI.transfer(command);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}
uint8_t* Magnetometer::ReadReg(uint8_t addressStart, uint8_t Data[], uint8_t Size) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    addressStart = addressStart | 0x80;
    addressStart = addressStart | 0x40;
    SPI.transfer(addressStart);
    for(int i = 0; i < Size; i++){
      Data[i] = SPI.transfer(Dummy);
    }

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return Data;
}
void Magnetometer::init() {
  WriteReg(CTRL_REG1, CTRL_REG1_Command);
  WriteReg(CTRL_REG2, CTRL_REG2_Command);
  WriteReg(CTRL_REG3, CTRL_REG3_Command);
  WriteReg(CTRL_REG4, CTRL_REG4_Command);
  WriteReg(CTRL_REG5, CTRL_REG5_Command);

}


void Magnetometer::readData(float &MagnetometerX, float &MagnetometerY, float &MagnetometerZ) {
  ReadReg(DataStart, MagData, MagDataSize);
  MagnetometerX = static_cast<float>(twosComplement(MagData[0], MagData[1])) / scale * 100;  // microTesla per gauss
  MagnetometerY = static_cast<float>(twosComplement(MagData[2], MagData[3])) / scale * 100;
  MagnetometerZ = static_cast<float>(twosComplement(MagData[4], MagData[5])) / scale * 100;
}

int16_t Magnetometer::twosComplement(uint8_t lowByte, uint8_t highByte) {
  // Combine the two bytes into a 16-bit signed integer
  int16_t result = (highByte << 8) | lowByte;
  
  // Check if the number is negative (if the most significant bit is set)
  if (result & 0x8000) {
    // Perform two's complement operation
    result = -((~result) + 1);
  }
  
  return result;
}
