#include "Arduino.h"
#include <SPI.h>
#include "bmi270_api/bmi270.h"

// Define SPI configuration variables
uint32_t IMUclockFrequency = 100000;
uint8_t  IMUCS    = 5;
uint8_t IMUMOSI  = 41;
uint8_t IMUMISO  = 40;
uint8_t IMUSCK   = 42;

// Create an instance of the BMI270 class
BMI270 bmi270;

void setup() {
  // Initialize SPI communication
  SPI.begin();
  SPI.beginTransaction(SPISettings(IMUclockFrequency, MSBFIRST, SPI_MODE3));

  // Initialize BMI270 over SPI
  int8_t result = bmi270.beginSPI(IMUCS, IMUSCK);

  if (result != BMI2_OK) {
    Serial.println("BMI270 initialization failed!");
    while (1);
  }

  // Other setup code as needed
}

void loop() {
  // Your main program logic here

  // Example: Get sensor data
  int8_t dataResult = bmi270.getSensorData();

  if (dataResult == BMI2_OK) {
    // Access sensor data using bmi270.data
    // Example: Print accelerometer data
    Serial.print("Accel X: "); Serial.print(bmi270.data.accelX);
    Serial.print(" Y: "); Serial.print(bmi270.data.accelY);
    Serial.print(" Z: "); Serial.println(bmi270.data.accelZ);
  } else {
    Serial.println("Failed to get sensor data!");
  }

  // Your main program logic here

  delay(1000); // Add a delay or adjust as needed
}
