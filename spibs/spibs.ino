
#include <SPI.h>

// Define your SPI pins
#define SCK_PIN  18  // SPI Clock
#define MISO_PIN 19  // Master In Slave Out
#define MOSI_PIN 23  // Master Out Slave In
#define SS_PIN   5   // Slave Select (Chip Select)

void setup() {
  // Start the SPI bus
  SPI.begin();

  // Configure the SPI settings
  SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);  // Adjust the parameters as needed
  SPI.beginTransaction(spiSettings);


Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

Serial.print("MOSI: ");
Serial.println(MOSI);
Serial.print("MISO: ");
Serial.println(MISO);
Serial.print("SCK: ");
Serial.println(SCK);
Serial.print("SS: ");
Serial.println(SS);

sleep(1000);
}

