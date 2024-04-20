#include "SPI.h"

// Define commands (page 10 of Datasheet)
const uint8_t MS5611_CMD_READ_ADC = 0x00;
const uint8_t MS5611_CMD_READ_PROM_FIRST = 0xA0;
const uint8_t MS5611_CMD_READ_PROM_LAST = 0xAE;
const uint8_t MS5611_CMD_RESET = 0x1E;
const uint8_t MS5611_CMD_CONVERT_D1 = 0x48; // OSR 4096
const uint8_t MS5611_CMD_CONVERT_D2 = 0x58; // OSR 4096

// Define Factory Constants
uint16_t Constants[8];

// Define data constants
uint32_t D1, D2, TEMP, P;

int64_t OFF, SENS;

bool _compensation = true;

// Define software SPI pin assignments
const int BarometerCS = 7;
const int BarometerMOSI = 41;
const int BarometerMISO = 40;
const int BarometerSCK = 42;

// Define custom SPISettings named barometerSPISettings
SPISettings barometerSPISettings(1000000, MSBFIRST, SPI_MODE0);

void setup() {
  Serial.begin(115200);

  pinMode(BarometerCS, OUTPUT);
  SPI.begin(BarometerSCK, BarometerMISO, BarometerMOSI, BarometerCS);
  delay(100);
  // Begin transaction with custom SPISettings
  SPI.beginTransaction(barometerSPISettings);
  digitalWrite(BarometerCS, LOW);

  // Send Reset command. Sent once after power-on to make sure that the calibration PROM gets loaded into the internal register
  SPI.transfer(MS5611_CMD_RESET);
  // Introduce a delay of 2.8 milliseconds. From datasheet pg 10
  delayMicroseconds(2800);

  digitalWrite(BarometerCS, HIGH);
  SPI.endTransaction();
  delay(100);
  /*
    Send Prom Command. Shall be executed once after reset by the user to read the content of the calibration PROM and to calculate the calibration coefficients.
    There are 8 total addresses resulting in a total memory of 128 bit. Address 0 contains factory data and the setup, addresses 1-6 calibration coefficients and address 7 contains
    the serial code and CRC. The command sequence is 8 bits long with a 16 bit result which is clocked with the MSB First
  */
  for (uint8_t command = MS5611_CMD_READ_PROM_FIRST; command <= MS5611_CMD_READ_PROM_LAST; command += 2) {
    SPI.beginTransaction(barometerSPISettings);
    digitalWrite(BarometerCS, LOW);

    // Send the command
    SPI.transfer(command);

    // Read the 16-bit value
    Constants[(command - MS5611_CMD_READ_PROM_FIRST) / 2] = SPI.transfer16(0);

    digitalWrite(BarometerCS, HIGH);
    SPI.endTransaction();
  }
}







void loop() {
  // Read raw sensor values
  D1 = barometerRead(MS5611_CMD_CONVERT_D1);
  D2 = barometerRead(MS5611_CMD_CONVERT_D2);

  // Perform compensation calculations by passing D1 and D2
  calculateCompensation(D1, D2);

  // Print compensated temperature and pressure to the serial monitor with decimal places
  Serial.print("Compensated Temperature: ");
  Serial.print(TEMP / 100.0); // Assuming two decimal places
  Serial.println(" degrees Celsius");

  Serial.print("Compensated Pressure: ");
  Serial.print(P / 100.0); // Assuming two decimal places
  Serial.println(" Pascals");

  delay(1000); 
}




//Reads the data from the barometer
uint32_t barometerRead(uint8_t command) {
  uint32_t result = 0;

  // Start the transaction
  SPI.beginTransaction(barometerSPISettings);

  // Send the command
  digitalWrite(BarometerCS, LOW);
  SPI.transfer(command);
  digitalWrite(BarometerCS, HIGH);
  // Introduce a delay of 8.22 milliseconds. From datasheet pg 11
  delayMicroseconds(8220);

  // Send the read ADC command
  digitalWrite(BarometerCS, LOW);
  SPI.transfer(MS5611_CMD_READ_ADC);

  // Get the first byte
  uint8_t byte1 = SPI.transfer(0);

  // Get the second byte
  uint8_t byte2 = SPI.transfer(0);

  // Get the third byte
  uint8_t byte3 = SPI.transfer(0);

  // Combine into a 32-bit int. It's a 24-bit number
  result = (static_cast<uint32_t>(byte1) << 16) |
           (static_cast<uint32_t>(byte2) << 8) |
           static_cast<uint32_t>(byte3);

  // End the transaction
  digitalWrite(BarometerCS, HIGH);
  SPI.endTransaction();

  // Return the result
  return result;
}

//Calculates the compensation for the barometer
void calculateCompensation(uint32_t D1, uint32_t D2) {
  // Assuming dT is int32_t with size 25 bits
  int32_t dT = D2 - (Constants[5] << 8);

  // Ensure that dT falls within the specified range
  dT = constrain(dT, -16776960, 16777216);

  // Assuming TEMP is signed 32-bit with size 41 bits
  TEMP = 2000 + ((int64_t)dT * Constants[6]) / (1 << 23);

  // Assuming OFF and SENS are int64_t with size 41 bits
  OFF = (Constants[2] * (1LL << 16)) + ((Constants[4] * dT) / (1 << 7));
  SENS = (Constants[1] * (1LL << 15)) + ((Constants[3] * dT) / (1 << 8));

  // Ensure that OFF and SENS fall within the specified ranges
  OFF = constrain(OFF, -8589672450LL, 12884705280LL);
  SENS = constrain(SENS, -4294836225LL, 6442352640LL);

  // Assuming P is int32_t with size 58 bits
  P = ((D1 * SENS) / (1LL << 21) - OFF) / (1 << 15);
}
