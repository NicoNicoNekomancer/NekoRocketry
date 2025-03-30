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
  // Apply hard iron correction first
  MagnetometerX -= magOffset[0];
  MagnetometerY -= magOffset[1];
  MagnetometerZ -= magOffset[2];
  // Store original values before soft iron correction
  float tempX = MagnetometerX;
  float tempY = MagnetometerY;
  float tempZ = MagnetometerZ;

  // Apply soft iron correction
  MagnetometerX = magSoftIron[0][0] * tempX + magSoftIron[0][1] * tempY + magSoftIron[0][2] * tempZ;
  MagnetometerY = magSoftIron[1][0] * tempX + magSoftIron[1][1] * tempY + magSoftIron[1][2] * tempZ;
  MagnetometerZ = magSoftIron[2][0] * tempX + magSoftIron[2][1] * tempY + magSoftIron[2][2] * tempZ;
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
    Serial.print("LOG:");
    Serial.println(diffX);
    Serial.print("LOG:");
    Serial.println(diffY);
    Serial.print("LOG:");
    Serial.println(diffZ);
    if ((diffX < 1 || diffX > 3) || (diffY < 1 || diffY > 3) || (diffZ < 0.1 || diffZ > 1)) {
      Serial.println("LOG:Fail");
    } else {
      Serial.println("LOG:Pass");
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


// Modified updateMinMax that avoids dramatic changes
void Magnetometer::updateMinMax(float mx, float my, float mz) {
  // Use a small adjustment factor to gradually update calibration
  const float adjustmentFactor = 0.01f;
  
  // Gradually update min values
  if (mx < minVal[0]) minVal[0] = minVal[0] * (1.0f - adjustmentFactor) + mx * adjustmentFactor;
  if (my < minVal[1]) minVal[1] = minVal[1] * (1.0f - adjustmentFactor) + my * adjustmentFactor;
  if (mz < minVal[2]) minVal[2] = minVal[2] * (1.0f - adjustmentFactor) + mz * adjustmentFactor;
  
  // Gradually update max values
  if (mx > maxVal[0]) maxVal[0] = maxVal[0] * (1.0f - adjustmentFactor) + mx * adjustmentFactor;
  if (my > maxVal[1]) maxVal[1] = maxVal[1] * (1.0f - adjustmentFactor) + my * adjustmentFactor;
  if (mz > maxVal[2]) maxVal[2] = maxVal[2] * (1.0f - adjustmentFactor) + mz * adjustmentFactor;
}

// Compute Hard Iron Bias from min/max values
void Magnetometer::computeHardIronBias() {
  for (int i = 0; i < 3; i++) {
    hardIronBias[i] = (maxVal[i] + minVal[i]) / 2.0f;
  }
}

// Compute Soft Iron Correction Matrix with safeguards
void Magnetometer::computeSoftIronMatrix() {
  float scale[3];
  for (int i = 0; i < 3; i++) {
    // Ensure we have a reasonable range to avoid division issues
    float range = maxVal[i] - minVal[i];
    scale[i] = (range > 0.1f) ? range / 2.0f : 1.0f;  // Default to 1.0 if range is too small
  }
  
  float avgScale = (scale[0] + scale[1] + scale[2]) / 3.0f;
  
  // Only update matrix elements with reasonable values
  for (int i = 0; i < 3; i++) {
    // Avoid division by zero or very small numbers
    if (scale[i] > 0.1f) {
      // Limit the correction factor to avoid extreme values
      float factor = avgScale / scale[i];
      factor = constrain(factor, 0.5f, 2.0f);  // Limit scaling to reasonable range
      softIronMatrix[i][i] = factor;
    }
  }
}

// Improved applyCalibration with safeguards
void Magnetometer::applyCalibration(float &mx, float &my, float &mz) {
  // Save original values for min/max update
  float origMx = mx;
  float origMy = my;
  float origMz = mz;
  
  // Only update calibration parameters occasionally to avoid constant flux
  static unsigned long lastCalibUpdate = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastCalibUpdate > 1000) {  // Update calibration once per second
    updateMinMax(origMx, origMy, origMz);
    computeHardIronBias();
    computeSoftIronMatrix();
    lastCalibUpdate = currentMillis;
  }
  
  // Remove hard iron bias
  float corrected[3] = {origMx - hardIronBias[0], origMy - hardIronBias[1], origMz - hardIronBias[2]};
  
  // Apply soft iron correction (only diagonal elements for simplicity and stability)
  mx = softIronMatrix[0][0] * corrected[0];
  my = softIronMatrix[1][1] * corrected[1];
  mz = softIronMatrix[2][2] * corrected[2];
  
  // Safety check to prevent near-zero values that would cause normalization issues
  float magSq = mx*mx + my*my + mz*mz;
  if (magSq < 0.01f) {
    // If magnitude is too small, use uncalibrated values but normalized
    magSq = origMx*origMx + origMy*origMy + origMz*origMz;
    if (magSq > 0.01f) {
      float scale = 0.1f / sqrt(magSq);
      mx = origMx * scale;
      my = origMy * scale; 
      mz = origMz * scale;
    } else {
      // Last resort: use default values
      mx = 0.1f;
      my = 0.0f;
      mz = 0.0f;
    }
  }
}

// Add this to your initialization code
void Magnetometer::initializeCalibration() {
  // Start with reasonable defaults
  for (int i = 0; i < 3; i++) {
    minVal[i] = -1.0f;  // Start with reasonable defaults instead of extreme values
    maxVal[i] = 1.0f;
    hardIronBias[i] = 0.0f;
  }
  
  // Initialize softIronMatrix to identity
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      softIronMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
}