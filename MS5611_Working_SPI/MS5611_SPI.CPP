#include "MS5611_SPI.h"

#define MS5611_CMD_READ_ADC       0x00
#define MS5611_CMD_READ_PROM      0xA0
#define MS5611_CMD_RESET          0x1E
#define MS5611_CMD_CONVERT_D1     0x40
#define MS5611_CMD_CONVERT_D2     0x50

MS5611_SPI::MS5611_SPI(uint8_t select, __SPI_CLASS__ * mySPI)
{
  _samplingRate = OSR_ULTRA_LOW;
  _temperature = MS5611_NOT_READ;
  _pressure = MS5611_NOT_READ;
  _result = MS5611_NOT_READ;
  _lastRead = 0;
  _deviceID = 0;
  _pressureOffset = 0;
  _temperatureOffset = 0;
  _compensation = true;

  _select = select;
  _dataIn = 255;
  _dataOut = 255;
  _clock = 255;
  _hwSPI = true;
  _mySPI = mySPI;
}

MS5611_SPI::MS5611_SPI(uint8_t select, uint8_t dataOut, uint8_t dataIn, uint8_t clock)
{
  _samplingRate = OSR_ULTRA_LOW;
  _temperature = MS5611_NOT_READ;
  _pressure = MS5611_NOT_READ;
  _result = MS5611_NOT_READ;
  _lastRead = 0;
  _deviceID = 0;
  _pressureOffset = 0;
  _temperatureOffset = 0;
  _compensation = false;

  _select = select;
  _dataIn = dataIn;
  _dataOut = dataOut;
  _clock = clock;
  _hwSPI = false;
  _mySPI = NULL;
}

bool MS5611_SPI::begin()
{
  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);

  setSPIspeed(_SPIspeed);

  if (_hwSPI)
  {
    _mySPI->begin();
    _mySPI->end();
    _mySPI->begin();
    delay(1);
  }
  else
  {
    pinMode(_dataIn, INPUT);
    pinMode(_dataOut, OUTPUT);
    pinMode(_clock, OUTPUT);
    digitalWrite(_dataOut, LOW);
    digitalWrite(_clock, LOW);
  }

  return reset();
}

bool MS5611_SPI::isConnected()
{
  int rv = read();
  return (rv == MS5611_READ_OK);
}

bool MS5611_SPI::reset(uint8_t mathMode)
{
  command(MS5611_CMD_RESET);
  uint32_t start = micros();
  while (micros() - start < 3000)
  {
    yield();
    delayMicroseconds(10);
  }

  initConstants(mathMode);

  bool ROM_OK = true;
  for (uint8_t reg = 0; reg < 7; reg++)
  {
    uint16_t tmp = readProm(reg);
    C[reg] *= tmp;
    _deviceID <<= 4;
    _deviceID ^= tmp;
    if (reg > 0)
    {
      ROM_OK = ROM_OK && (tmp != 0);
    }
  }
  return ROM_OK;
}

int MS5611_SPI::read(uint8_t bits)
{
  convert(MS5611_CMD_CONVERT_D1, bits);
  uint32_t _D1 = readADC();
  convert(MS5611_CMD_CONVERT_D2, bits);
  uint32_t _D2 = readADC();

  float dT = _D2 - C[5];
  _temperature = 2000 + dT * C[6];

  float offset = C[2] + dT * C[4];
  float sens = C[1] + dT * C[3];

  if (_compensation)
  {
    if (_temperature < 2000)
    {
      float T2 = dT * dT * 4.6566128731E-10;
      float t = (_temperature - 2000) * (_temperature - 2000);
      float offset2 = 2.5 * t;
      float sens2 = 1.25 * t;
      if (_temperature < -1500)
      {
        t = (_temperature + 1500) * (_temperature + 1500);
        offset2 += 7 * t;
        sens2 += 5.5 * t;
      }
      _temperature -= T2;
      offset -= offset2;
      sens -= sens2;
    }
  }

  _pressure = (_D1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-5;

  _lastRead = millis();
  return MS5611_READ_OK;
}

void MS5611_SPI::setOversampling(osr_t samplingRate)
{
  _samplingRate = (uint8_t)samplingRate;
}

osr_t MS5611_SPI::getOversampling() const
{
  return (osr_t)_samplingRate;
}

float MS5611_SPI::getTemperature() const
{
  if (_temperatureOffset == 0)
    return _temperature * 0.01;
  return _temperature * 0.01 + _temperatureOffset;
}

float MS5611_SPI::getPressure() const
{
  if (_pressureOffset == 0)
    return _pressure * 0.01;
  return _pressure * 0.01 + _pressureOffset;
}

void MS5611_SPI::setPressureOffset(float offset)
{
  _pressureOffset = offset;
}

float MS5611_SPI::getPressureOffset()
{
  return _pressureOffset;
}

void MS5611_SPI::setTemperatureOffset(float offset)
{
  _temperatureOffset = offset;
}

float MS5611_SPI::getTemperatureOffset()
{
  return _temperatureOffset;
}

int MS5611_SPI::getLastResult() const
{
  return _result;
}

uint32_t MS5611_SPI::lastRead() const
{
  return _lastRead;
}

uint32_t MS5611_SPI::getDeviceID() const
{
  return _deviceID;
}

void MS5611_SPI::setCompensation(bool flag)
{
  _compensation = flag;
}

bool MS5611_SPI::getCompensation()
{
  return _compensation;
}

uint16_t MS5611_SPI::getManufacturer()
{
  return readProm(0);
}

uint16_t MS5611_SPI::getSerialCode()
{
  return readProm(7) >> 4;
}

void MS5611_SPI::setSPIspeed(uint32_t speed)
{
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
}

uint32_t MS5611_SPI::getSPIspeed()
{
  return _SPIspeed;
}

bool MS5611_SPI::usesHWSPI()
{
  return _hwSPI;
}

void MS5611_SPI::convert(const uint8_t addr, uint8_t bits)
{
  uint16_t del[5] = {600, 1200, 2300, 4600, 9100};
  uint8_t index = bits;
  if (index > 4)
    index = 4;
  command(addr);
  delayMicroseconds(del[index]);
}

uint32_t MS5611_SPI::readADC()
{
  command(MS5611_CMD_READ_ADC);
  delayMicroseconds(100);
  return (uint32_t)command(0) << 16 | (uint32_t)command(0) << 8 | command(0);
}

uint16_t MS5611_SPI::readProm(uint8_t reg)
{
  if (reg > 6)
    return 0;
  return command(MS5611_CMD_READ_PROM | (reg << 1)) << 8 | command(MS5611_CMD_READ_PROM | (reg << 1) | 1);
}

int MS5611_SPI::command(const uint8_t command)
{
  if (_hwSPI)
  {
    _mySPI->beginTransaction(_spi_settings);
    digitalWrite(_select, LOW);
    _result = _mySPI->transfer(command);
    digitalWrite(_select, HIGH);
    _mySPI->endTransaction();
  }
  else
  {
    _result = swSPI_transfer(command);
  }
  return _result;
}

uint8_t MS5611_SPI::swSPI_transfer(uint8_t value)
{
  uint8_t result = 0;
  for (uint8_t i = 0; i < 8; ++i)
  {
    digitalWrite(_clock, HIGH);
    if (value & 0x80)
    {
      digitalWrite(_dataOut, HIGH);
    }
    else
    {
      digitalWrite(_dataOut, LOW);
    }
    value <<= 1;
    digitalWrite(_clock, LOW);
    result <<= 1;
    result |= digitalRead(_dataIn);
  }
  return result;
}
