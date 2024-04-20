#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include <SPI.h>

class IMU {
public:
    IMU(int csPin, int mosiPin, int misoPin, int sckPin, int clock);
    void begin();
    void readData(float &AccelX, float &AccelY, float &AccelZ, float &GyroX, float &GyroY, float &GyroZ, float &IMUTemp);

private:

    int _csPin, _mosiPin, _misoPin, _sckPin;
    SPISettings _spiSettings;
    uint8_t  configcheck = 0;
    uint8_t  IMUData[25];
    uint16_t LSB8G = 4096;
    float    LSB2000 = 16.4;
    int16_t  AccelXRAW, AccelYRAW, AccelZRAW, GyroXRAW, GyroYRAW, GyroZRAW;
    uint8_t  GYR_CAS, TempRaw1, TempRaw2;
    int8_t   GYR_CAS_Twos;

    float    AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, Temp;

   const uint8_t chip_ID = 0x00;
    const uint8_t dummy = 0x00;
    const uint8_t bitflip = 0x80;
    const uint8_t CMD = 0x7E;
    const uint8_t powerCONF = 0x7C;
    const uint8_t ACC_RANGE = 0x41;
    const uint8_t GYR_RANGE = 0x43;
    const uint8_t init_control= 0x59;
    const uint8_t init_ADDR_0 = 0x5b;
    const uint8_t init_array[2] = {0x00, 0x00};
    const uint8_t three_array[3] = {0x00, 0x00, 0x00};
    const uint8_t features_array[17] = {0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00};

    uint8_t readReg(uint8_t regAddress);
    void writeRegCont(uint8_t regAddress, const uint8_t data[], uint8_t array_size);
    void writeRegConfig();
    void writeReg(uint8_t regAddress, uint8_t data);
    void testcommunication();
    void initIMU();
    void getIMUData();
    int16_t twosComplement(uint8_t lowByte, uint8_t highByte);
};

#endif
