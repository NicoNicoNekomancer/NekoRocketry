#include "Barometer.h"

Barometer::Barometer(int csPin, int mosiPin, int misoPin, int sckPin, int clock) {
    _csPin = csPin;
    _mosiPin = mosiPin;
    _misoPin = misoPin;
    _sckPin = sckPin;
    _spiSettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
}

void Barometer::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    //SPI.setCS(_csPin);
    SPI.setRX(_misoPin);
    SPI.setTX(_mosiPin);
    SPI.setSCK(_sckPin);
    SPI.begin();
    delay(100);
    reset();
    readProm();
}

void Barometer::reset() {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(MS5611_CMD_RESET);
    delayMicroseconds(3000);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    delay(100);
}

void Barometer::readProm() {
    for (uint8_t command = MS5611_CMD_READ_PROM_FIRST; command <= MS5611_CMD_READ_PROM_LAST; command += 2) {
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_csPin, LOW);
        SPI.transfer(command);
        _constants[(command - MS5611_CMD_READ_PROM_FIRST) / 2] *= SPI.transfer16(0);
        digitalWrite(_csPin, HIGH);
        SPI.endTransaction();
        
    }
}

uint32_t Barometer::readAdc(uint8_t command) {
    uint32_t result = 0;
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(command);
    delayMicroseconds(8220);
    digitalWrite(_csPin, HIGH);
    digitalWrite(_csPin, LOW);
    SPI.transfer(MS5611_CMD_READ_ADC);
    uint8_t byte1 = SPI.transfer(0);
    uint8_t byte2 = SPI.transfer(0);
    uint8_t byte3 = SPI.transfer(0);
    result = (static_cast<uint32_t>(byte1) << 16) |
             (static_cast<uint32_t>(byte2) << 8) |
             static_cast<uint32_t>(byte3);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return result;
}

void Barometer::calculateCompensation(uint32_t D1, uint32_t D2) {
    dT = D2 - _constants[5];
    dT = constrain(dT, -16776960, 16777216);
    TEMP = 2000 + dT * _constants[6];

    OFF = _constants[2] + _constants[4] * dT;
    SENS = _constants[1] + _constants[3] * dT;

    if(TEMP < 2000){
      T2 = dT*dT / 2147483648;
      t = (TEMP - 2000) * (TEMP - 2000);
      OFF2 = 2.5 * t;
      SENS2 = 1.25 * t;
      if(TEMP < -1500){
        t = (TEMP + 1500) * (TEMP + 1500);
        OFF2 += 7 * t ;
        SENS2 += 5.5 * t;
      }
    }else{
      T2 = 0;
      OFF2 = 0;
      SENS2 = 0;
    }
    TEMP  -= T2;
    OFF   -= OFF2;
    SENS  -= SENS2;
  
    OFF  = constrain(OFF, -8589672450LL, 12884705280LL);
    SENS = constrain(SENS, -4294836225LL, 6442352640LL);

    P = (D1 * SENS / 2097152 - OFF) / 32768;
}
void Barometer::readData(float &temperature, float &pressure) {
    uint32_t D1 = readAdc(MS5611_CMD_CONVERT_D1);
    uint32_t D2 = readAdc(MS5611_CMD_CONVERT_D2);
    calculateCompensation(D1, D2);

    // Assign values to temperature and pressure
    temperature = TEMP / 100.0; // Assuming two decimal places
    pressure = P / 100.0; // Assuming two decimal places
}
