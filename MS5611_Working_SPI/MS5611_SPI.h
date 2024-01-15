#pragma once

#include "Arduino.h"
#include "SPI.h"

#define MS5611_SPI_LIB_VERSION (F("0.2.0 EXPERIMENTAL"))

#ifndef __SPI_CLASS__
#if defined(ARDUINO_ARCH_RP2040)
#define __SPI_CLASS__ SPIClassRP2040
#else
#define __SPI_CLASS__ SPIClass
#endif
#endif

#define MS5611_READ_OK 0
#define MS5611_ERROR_2 2 // TODO ??
#define MS5611_NOT_READ -999

enum osr_t
{
  OSR_ULTRA_HIGH = 12,
  OSR_HIGH = 11,
  OSR_STANDARD = 10,
  OSR_LOW = 9,
  OSR_ULTRA_LOW = 8
};

class MS5611_SPI
{
public:
  explicit MS5611_SPI(uint8_t select, __SPI_CLASS__ *mySPI = &SPI);
  explicit MS5611_SPI(uint8_t select, uint8_t dataOut, uint8_t dataIn, uint8_t clock);

  bool begin();
  bool isConnected();
  bool reset(uint8_t mathMode = 0);
  int read(uint8_t bits);
  inline int read() { return read((uint8_t)_samplingRate); };
  void setOversampling(osr_t samplingRate);
  osr_t getOversampling() const;
  float getTemperature() const;
  float getPressure() const;
  void setPressureOffset(float offset = 0);
  float getPressureOffset();
  void setTemperatureOffset(float offset = 0);
  float getTemperatureOffset();
  int getLastResult() const;
  uint32_t lastRead() const;
  uint32_t getDeviceID() const;
  void setCompensation(bool flag = true);
  bool getCompensation();
  uint16_t getManufacturer();
  uint16_t getSerialCode();
  void setSPIspeed(uint32_t speed);
  uint32_t getSPIspeed();
  bool usesHWSPI();

protected:
  void convert(const uint8_t addr, uint8_t bits);
  uint32_t readADC();
  uint16_t readProm(uint8_t reg);
  int command(const uint8_t command);
  void initConstants(uint8_t mathMode);

  uint8_t _address;
  uint8_t _samplingRate;
  int32_t _temperature;
  int32_t _pressure;
  float _pressureOffset;
  float _temperatureOffset;
  int _result;
  float C[7];
  uint32_t _lastRead;
  uint32_t _deviceID;
  bool _compensation;
  uint8_t _select;
  uint8_t _dataIn;
  uint8_t _dataOut;
  uint8_t _clock;
  bool _hwSPI;
  uint32_t _SPIspeed = 1000000;
  uint8_t swSPI_transfer(uint8_t value);
  __SPI_CLASS__ *_mySPI;
  SPISettings _spi_settings;
};
