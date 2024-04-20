#include "BMI270.h"

BMI270::BMI270(uint8_t csPin) : csPin(csPin) {}

void BMI270::begin() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    SPI.begin();
    testcommunication();
    initIMU();
}

void BMI270::getIMUData(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ) {
    uint8_t IMUData[25];
    int16_t AccelXRAW, AccelYRAW, AccelZRAW, GyroXRAW, GyroYRAW, GyroZRAW;
    uint8_t GYR_CAS;
    int8_t GYR_CAS_Twos;

    getIMUDataRaw(IMUData, sizeof(IMUData));
    
    GYR_CAS = readReg(0x3C);
    GYR_CAS_Twos = (GYR_CAS & 0x40) ? (GYR_CAS | 0x80) : GYR_CAS;

    AccelXRAW = twosComplement(IMUData[10], IMUData[11]);
    accelX = static_cast<float>(AccelXRAW) / LSB8G;
    AccelYRAW = twosComplement(IMUData[12], IMUData[13]);
    accelY = static_cast<float>(AccelYRAW) / LSB8G;
    AccelZRAW = twosComplement(IMUData[14], IMUData[15]);
    accelZ = static_cast<float>(AccelZRAW) / LSB8G;

    GyroYRAW = twosComplement(IMUData[18], IMUData[19]);
    gyroY = static_cast<float>(GyroYRAW) / LSB2000;
    GyroZRAW = twosComplement(IMUData[20], IMUData[21]);
    gyroZ = static_cast<float>(GyroZRAW) / LSB2000;

    GyroXRAW = twosComplement(IMUData[16], IMUData[17]);
    GyroXRAW = GyroXRAW - (GYR_CAS_Twos * GyroZRAW / 512);
    gyroX = static_cast<float>(GyroXRAW) / LSB2000;
}

void BMI270::testcommunication() {
    uint8_t chipID = 0;
    while (chipID == 0) {
        readReg(chip_ID);
        chipID = readReg(chip_ID);
        if (chipID != 0x24) {
            delay(200);
        }
        writeReg(CMD, 0xB6);
        delayMicroseconds(2000);
    }
}

void BMI270::initIMU() {
    readReg(chip_ID);
    readReg(powerCONF);
    writeReg(powerCONF, 0x00);
    delayMicroseconds(450);
    readReg(init_control);
    writeReg(init_control, 0x00);
    writeRegConfig(); 
    delayMicroseconds(450);
    readReg(init_control);
    writeReg(init_control, 0x01);
    delayMicroseconds(450);
    configcheck = readReg(0x21);
    if (configcheck == 1) {
        // Config has loaded
    } else {
        // Config has failed to load
    }
    writeReg(0x7D, 0x0E);
    writeReg(0x40, 0xEC);
    writeReg(0x42, 0xDD);
    writeReg(ACC_RANGE, 0x02);
    writeReg(GYR_RANGE, 0x08);
    writeReg(0x7C, 0x02);
}

void BMI270::writeReg(uint8_t regAddress, uint8_t data) {
    SPI.beginTransaction(SPISettings(clockFrequency, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin, LOW);
    SPI.transfer(regAddress);
    SPI.transfer(data);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(16);
}

uint8_t BMI270::readReg(uint8_t regAddress) {
    SPI.beginTransaction(SPISettings(clockFrequency, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin, LOW);
    SPI.transfer((regAddress | bitflip));
    SPI.transfer(dummy);
    uint8_t result = SPI.transfer(dummy);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(16);
    return result;
}

void BMI270::writeRegConfig() {
    int array_size = sizeof(bmi270_maximum_fifo_config_file);
    int section = array_size / 32;
    for (int i = 0; i < section; i++) {
        SPI.beginTransaction(SPISettings(clockFrequency, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin, LOW);
        SPI.transfer(0x5B);
        delayMicroseconds(16);
        SPI.transfer(0x00);
        delayMicroseconds(16);
        SPI.transfer(0x00+ i);
        delayMicroseconds(16);
        digitalWrite(csPin, HIGH);
        SPI.endTransaction();
        delayMicroseconds(16);
        SPI.beginTransaction(SPISettings(clockFrequency, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin, LOW);
        SPI.transfer(0x5E);
        for (int z = 0; z < 32; z++) {
            SPI.transfer(bmi270_maximum_fifo_config_file[z + (32 * i)]);
            delayMicroseconds(16);
        }
        digitalWrite(csPin, HIGH);
        SPI.endTransaction();
    }
    if (array_size % 32 != 0) {
        SPI.beginTransaction(SPISettings(clockFrequency, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin, LOW);
        SPI.transfer(0x5B);
        SPI.transfer(0x00);
        SPI.transfer(0x00 + section);
        digitalWrite(csPin, HIGH);
        SPI.endTransaction();
        SPI.beginTransaction(SPISettings(clockFrequency, MSBFIRST, SPI_MODE0));
        digitalWrite(csPin, LOW);
        for (int i = 0; i < array_size - (section * 32); i++) {
            SPI.transfer(bmi270_maximum_fifo_config_file[i + (32 * section)]);
            delayMicroseconds(16);
        }
        digitalWrite(csPin, HIGH);
        SPI.endTransaction();
    }
}

void BMI270::getIMUDataRaw(uint8_t data[], uint8_t size) {
    SPI.beginTransaction(SPISettings(clockFrequency , MSBFIRST, SPI_MODE0));
    digitalWrite(csPin, LOW);
    SPI.transfer(0x83);
    for (int i = 0; i < size; i++) {
        data[i] = SPI.transfer(dummy);
    }
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(16);
}

int16_t BMI270::twosComplement(uint8_t lowByte, uint8_t highByte) {
    int16_t result = (highByte << 8) | lowByte;
    if (result & 0x8000) {
        result = -((~result) + 1);
    }
    return result;
}
