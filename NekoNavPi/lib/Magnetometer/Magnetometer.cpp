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
    //SPI.setCS(_csPin);
    SPI.setRX(_misoPin);
    SPI.setTX(_mosiPin);
    SPI.setSCK(_sckPin);
    SPI.begin();
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
    addressStart |= 0xC0;  // 0x80 | 0x40
    SPI.transfer(addressStart);
    for(int i = 0; i < Size; i++){
      Data[i] = SPI.transfer(Dummy);
    }

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return Data;
}
uint8_t Magnetometer::ReadStatus(uint8_t Data) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(0x27 | 0x80);
    Data = SPI.transfer(Dummy);
  

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


void Magnetometer::readData(float &MagnetometerX, float &MagnetometerY, float &MagnetometerZ, float magOffset[3], float magSoftIron[3][3]) {
  ReadReg(DataStart, MagData, MagDataSize);
  MagnetometerX = static_cast<float>(twosComplement(MagData[0], MagData[1])) * scale;  
  MagnetometerY = static_cast<float>(twosComplement(MagData[2], MagData[3])) * scale;
  MagnetometerZ = static_cast<float>(twosComplement(MagData[4], MagData[5])) * scale;

  float magnetometer_reading[3] = {MagnetometerX, MagnetometerY, MagnetometerZ};
  float calibrated_reading[3];
}

int16_t Magnetometer::twosComplement(uint8_t lowByte, uint8_t highByte) {
    // Combine the two bytes into a 16-bit signed integer
    int16_t result = (static_cast<int16_t>(highByte) << 8) | lowByte;

    // Perform two's complement operation (if the most significant bit is set)
    result = (result & 0x8000) ? -((~result) + 1) : result;

    return result;
}

void Magnetometer::SelfTest(){
    WriteReg(0x20, 0x1C);
    WriteReg(0x21, 0x40);
    delay(20);
    WriteReg(0x22, 0x00);
    delay(20);
    while((ReadStatus(Status) & 0x01) != 1){}
    ReadReg(DataStart, SelfTestData, MagDataSize);
    SelfTestX1 = static_cast<float>(twosComplement(SelfTestData[0], SelfTestData[1])) / 2281;  
    SelfTestY1 = static_cast<float>(twosComplement(SelfTestData[2], SelfTestData[3])) / 2281;
    SelfTestZ1 = static_cast<float>(twosComplement(SelfTestData[4], SelfTestData[5])) / 2281;

    for(int i = 0; i < 5; i++){
      while((ReadStatus(Status) & 0x01) != 1){}
      ReadReg(DataStart, SelfTestData, MagDataSize);
      SelfTestX1 += static_cast<float>(twosComplement(SelfTestData[0], SelfTestData[1])) / 2281;  
      SelfTestY1 += static_cast<float>(twosComplement(SelfTestData[2], SelfTestData[3])) / 2281;
      SelfTestZ1 += static_cast<float>(twosComplement(SelfTestData[4], SelfTestData[5])) / 2281;
    }
    SelfTestX1 /= 5;
    SelfTestY1 /= 5;
    SelfTestZ1 /= 5;
    WriteReg(0x20, 0x1D);
    while((ReadStatus(Status) & 0x01) != 1){}
    ReadReg(DataStart, SelfTestData, MagDataSize);
    SelfTestX2 = static_cast<float>(twosComplement(SelfTestData[0], SelfTestData[1])) / 2281;  
    SelfTestY2 = static_cast<float>(twosComplement(SelfTestData[2], SelfTestData[3])) / 2281;
    SelfTestZ2 = static_cast<float>(twosComplement(SelfTestData[4], SelfTestData[5])) / 2281;

    for(int i = 0; i < 5; i++){
      while((ReadStatus(Status) & 0x01) != 1){}
      ReadReg(DataStart, SelfTestData, MagDataSize);
      SelfTestX2 += static_cast<float>(twosComplement(SelfTestData[0], SelfTestData[1])) / 2281;  
      SelfTestY2 += static_cast<float>(twosComplement(SelfTestData[2], SelfTestData[3])) / 2281;
      SelfTestZ2 += static_cast<float>(twosComplement(SelfTestData[4], SelfTestData[5])) / 2281;
    }
    SelfTestX2 /= 5;
    SelfTestY2 /= 5;
    SelfTestZ2 /= 5;

    diffX = abs(SelfTestX2 - SelfTestX1);
    diffY = abs(SelfTestY2 - SelfTestY1);
    diffZ = abs(SelfTestZ2 - SelfTestZ1);
    Serial.println(diffX);
    Serial.println(diffY);
    Serial.println(diffZ);
    if ((diffX < 1 || diffX > 3) || (diffY < 1 || diffY > 3) || (diffZ < 0.1 || diffZ > 1)) {
      Serial.println("Fail");
    } else {
      Serial.println("Pass");
    }
    WriteReg(0x20, 0x1C);
}

void Magnetometer::storeOffset(float gaussValueX, float gaussValueY, float gaussValueZ){
  // Unscale the value
  scale /= scale;
  int16_t rawValueX = static_cast<int16_t>(gaussValueX * scale);
  int16_t rawValueY = static_cast<int16_t>(gaussValueY * scale);
  int16_t rawValueZ = static_cast<int16_t>(gaussValueZ * scale);
  // Convert to two's complement if negative
  if (rawValueX < 0) rawValueX = ~(-rawValueX) + 1;
  if (rawValueY < 0) rawValueY = ~(-rawValueY) + 1;
  if (rawValueZ < 0) rawValueZ = ~(-rawValueZ) + 1;


  // Write to the registers
    WriteReg(0x05, rawValueX & 0xFF);
    WriteReg(0x06, (rawValueX >> 8) & 0xFF);
    WriteReg(0x07, rawValueY & 0xFF);
    WriteReg(0x08, (rawValueY >> 8) & 0xFF);
    WriteReg(0x09, rawValueZ & 0xFF);
    WriteReg(0x0A, (rawValueZ >> 8) & 0xFF);
}