#include <SPI.h>

#define CS_PIN    5
#define IMUCS     5
#define IMUMOSI   41
#define IMUMISO   40
#define IMUSCK    42

void setup() {
  Serial.begin(115200);
  SPI.begin();

  pinMode(CS_PIN, OUTPUT);

  // Initialize BMI270
  initializeBMI270();
}

void loop() {
  // Read accelerometer and gyroscope data
  int16_t accelX = readSensorData(0x12); // Register address for accelerometer X data
  int16_t accelY = readSensorData(0x14); // Register address for accelerometer Y data
  int16_t accelZ = readSensorData(0x16); // Register address for accelerometer Z data
  int16_t gyroX = readSensorData(0x02);  // Register address for gyroscope X data
  int16_t gyroY = readSensorData(0x04);  // Register address for gyroscope Y data
  int16_t gyroZ = readSensorData(0x06);  // Register address for gyroscope Z data

  // Print the sensor data
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(accelX); Serial.print(", ");
  Serial.print("Y = "); Serial.print(accelY); Serial.print(", ");
  Serial.print("Z = "); Serial.println(accelZ);

  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(gyroX); Serial.print(", ");
  Serial.print("Y = "); Serial.print(gyroY); Serial.print(", ");
  Serial.print("Z = "); Serial.println(gyroZ);

  delay(1000); // Adjust the delay based on your application requirements
}

void initializeBMI270() {
  // Check the chip ID to verify communication
  uint8_t chipID = readRegister(0x00); // Register address for CHIP_ID
  if (chipID == 0x24) {
    Serial.println("BMI270 sensor detected!");
  } else {
    Serial.println("Error: Unable to detect BMI270 sensor. Check connections.");
    while (1);
  }

  // Soft reset
  writeRegister(0x7E, 0xB6);

  // Wait for the sensor to reset
  delay(100);

  // Configure accelerometer
  writeRegister(0x40, 0x1C); // Set accelerometer range and bandwidth

  // Configure gyroscope
  writeRegister(0x42, 0x0C); // Set gyroscope range and bandwidth

  // Enable accelerometer and gyroscope
  writeRegister(0x4B, 0x00);

  // You can perform additional configurations based on your requirements
}

int16_t readSensorData(uint8_t regAddr) {
  // Read two bytes of data from the specified register
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(regAddr | 0x80); // Set MSB to 1 for read operation
  int16_t data = (SPI.transfer(0) << 8) | SPI.transfer(0);
  digitalWrite(CS_PIN, HIGH);

  return data;
}

void writeRegister(uint8_t regAddr, uint8_t data) {
  // Write one byte to the specified register
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(regAddr & 0x7F); // Set MSB to 0 for write operation
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t readRegister(uint8_t regAddr) {
  // Read one byte of data from the specified register
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(regAddr | 0x80); // Set MSB to 1 for read operation
  uint8_t data = SPI.transfer(0);
  digitalWrite(CS_PIN, HIGH);

  return data;
}
