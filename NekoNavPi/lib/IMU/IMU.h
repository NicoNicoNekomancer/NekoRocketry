//ngl this chip kinda sucks ass because it dosent really have a good datasheet, but oh well
//currently doing +-8G 2000DPS
//check datasheet to change

//https://www.mouser.be/datasheet/2/783/bst_bmi270_ds000-2529306.pdf


//data is output in g's and rad/s


#ifndef IMU_h
#define IMU_h
#include <Arduino.h>
#include <SPI.h>


class IMU {
public:
    IMU(int csPin, int mosiPin, int misoPin, int sckPin, int clock);
    void begin();
    void readData(float &AccelX, float &AccelY, float &AccelZ, float &GyroX, float &GyroY, float &GyroZ, float &IMUTemp);
    void remapAxisSign(uint8_t axis);

private:

    int _csPin, _mosiPin, _misoPin, _sckPin;
    SPISettings _spiSettings;
    uint8_t  configcheck = 0;
    uint8_t  IMUData[25];
    float    LSB8G = 0.000244140625;  // 1/4096
    float    LSB16G = 0.00048828125;  // 1/2048
    float    LSB2000 = 0.06103515625; // 1/16.384
    int16_t  AccelXRAW, AccelYRAW, AccelZRAW, GyroXRAW, GyroYRAW, GyroZRAW;
    uint8_t  GYR_CAS, TempRaw1, TempRaw2;
    int8_t   GYR_CAS_Twos;

    float    AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, Temp;

    const uint8_t chip_ID = 0x00;
    const uint8_t dummy = 0x00;
    const uint8_t bitflip = 0x80;
    const uint8_t CMD = 0x7E;
    const uint8_t powerCONF = 0x7C;
    const uint8_t ACC_CONF = 0x40;
    const uint8_t ACC_RANGE = 0x41;
    const uint8_t GYR_CONF = 0x42;
    const uint8_t GYR_RANGE = 0x43;
    const uint8_t init_control= 0x59;
    const uint8_t init_ADDR_0 = 0x5b;
    const uint8_t init_array[2] = {0x00, 0x00};
    const uint8_t three_array[3] = {0x00, 0x00, 0x00};
    const uint8_t features_array[17] = {0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00};

    uint8_t readReg(uint8_t regAddress);
    uint16_t readRegPage(uint8_t regAddress, uint8_t page);
    void writeRegPage(uint8_t regAddress, uint8_t page, uint8_t data1, uint8_t data2);
    void writeRegCont(uint8_t regAddress, const uint8_t data[], uint8_t array_size);
    void writeRegConfig();
    void writeReg(uint8_t regAddress, uint8_t data);
    void testcommunication();
    void initIMU();
    void getIMUData();
    
    int16_t twosComplement(uint8_t lowByte, uint8_t highByte);
};

#endif
