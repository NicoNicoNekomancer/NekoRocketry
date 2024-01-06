/*
   RadioLib SX126x Ping-Pong Example

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"

BMI270 imu;
#define INITIATING_NODE
//setting pins being used
int LORACS    = 6; //shared between spi and lora
int LORAIRQ   = 2;
int LORARST   = 3;
int LORABUSY  = 4;
int LORAMOSI  = 13;
int LORAMISO  = 19;
int LORASCK   = 18;

uint8_t chipSelectPin = 5;
uint32_t clockFrequency = 100000;
int IMUCS     = 5;
int IMUMOSI  = 41;
int IMUMISO  = 40; //adr
int IMUSCK   = 42;

int DataLength = 0;
String message = String(100, BIN);

// SX1262 has the following connections:
// NSS pin:   cs
// DIO1 pin:  irq
// NRST pin:  rst
// BUSY pin:  gpio
// SPI 
// SPI Settings
SPIClass LoraSPI(HSPI);
SPISettings LoraSPISettings(2000000, MSBFIRST, SPI_MODE0);
LLCC68 radio = new Module(LORACS, LORAIRQ, LORARST, LORABUSY, LoraSPI, LoraSPISettings); // this one must be used because esp32



// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // we sent or received a packet, set the flag
  operationDone = true;
}

void setup() {
    pinMode(IMUCS, OUTPUT);
    pinMode(LORACS, OUTPUT);
    digitalWrite(LORACS, HIGH);
    digitalWrite(IMUCS, HIGH);

  Serial.begin(115200);

    Serial.println("BMI270 Example 2 - Basic Readings SPI");

    // Initialize the SPI library
    SPI.begin(IMUSCK,IMUMISO,IMUMOSI, IMUCS);

    
    imu.beginSPI(IMUCS, clockFrequency);
    
    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
    while(imu.beginSPI(IMUCS, clockFrequency) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }
  Serial.println("BMI270 connected!");

  //(SCK, MISO, MOSI, CS)
  LoraSPI.begin(LORASCK,LORAMISO,LORAMOSI,LORACS);
  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));


  // carrier frequency:           Carrier frequency in MHz. Defaults to 434.0 MHz. The .0 is important because otherwise it freaks out and goes back to default
  // bandwidth:                   LoRa bandwidth in kHz. Defaults to 125.0 kHz. Up to 500 kHz. Will need to revisit datasheet to get the exact numbers
  // spreading factor:            LoRa spreading factor. Defaults to 9. {5-11}
  // coding rate:                 LoRa coding rate denominator. Defaults to 7 (coding rate 4/7) {5-8}
  // sync word:                   1-byte LoRa sync word. Defaults to RADIOLIB_SX126X_SYNC_WORD_PRIVATE (0x12). 0x34 (public network/LoRaWAN). Will add more here as I continue to learn how this works
  // output power:                Output power in dBm. Defaults to 10 dBm. Can go up to 22
  // preamble length:             LoRa preamble length in symbols. Defaults to 8 symbols.
  //tcxoVoltage	                  TCXO reference voltage to be set on DIO3. Defaults to 1.6 V. If you are seeing -706/-707 error codes, it likely means you are using non-0 value for module with XTAL. To use XTAL, either set this value to 0, or set SX126x::XTAL to true.
  //useRegulatorLDO	              Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.
  int state = radio.begin(420.0, 500.0, 11, 5, 0x34, 20, 8, 1.6, false);
  

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
    Serial.print(F("[SX1262] Sending first packet ... "));
    transmissionState = radio.startTransmit("Sending Intial Handshake");
    transmitFlag = true;

}
void loop() {
    
      digitalWrite(IMUCS, HIGH);
    Serial.println(digitalRead(IMUCS));
    // Get measurements from the sensor. This must be called before accessing
    // the sensor data, otherwise it will never update
    imu.getSensorData();
    Serial.println(digitalRead(IMUCS));
    // Print acceleration data
    Serial.print("Acceleration in g's");
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(imu.data.accelX, 3);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(imu.data.accelY, 3);
    Serial.print("\t");
    Serial.print("Z: ");
    Serial.print(imu.data.accelZ, 3);

    Serial.print("\t");

    // Print rotation data
    Serial.print("Rotation in deg/sec");
    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(imu.data.gyroX, 3);
    Serial.print("\t");
    Serial.print("Y: ");
    Serial.print(imu.data.gyroY, 3);
    Serial.print("\t");
    Serial.print("Z: ");
    Serial.println(imu.data.gyroZ, 3);


      DataLength = message.length();
      Serial.println(F("[SX1262] Sending another packet ... "));
      transmissionState = radio.startTransmit(message, DataLength);
      Serial.println(digitalRead(LORACS));
      transmitFlag = true;
      Serial.println(digitalRead(LORACS));
      delay(500);
  }

