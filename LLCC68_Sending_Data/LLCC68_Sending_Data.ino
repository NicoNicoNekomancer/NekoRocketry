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
uint32_t LORAclockFrequency = 2000000;
uint8_t LORACS    = 6; //shared between spi and lora
uint8_t LORAIRQ   = 2;
uint8_t LORARST   = 3;
uint8_t LORABUSY  = 4;
uint16_t LORAMOSI  = 13;
uint16_t LORAMISO  = 19;
uint16_t LORASCK   = 18;


uint32_t IMUclockFrequency = 100000;
uint8_t  IMUCS    = 5;
uint16_t IMUMOSI  = 41;
uint16_t IMUMISO  = 40; //adr
uint16_t IMUSCK   = 42;
//initialize sensor variables
float AccelX = 0;
float AccelY = 0;
float AccelZ = 0;
float GyroX  = 0;
float GyroY  = 0;
float GyroZ  = 0;
//Initialize message variables
uint8_t DataLength = 0;
String message = String(100, BIN);

// SX1262 has the following connections:
// NSS pin:   cs
// DIO1 pin:  irq
// NRST pin:  rst
// BUSY pin:  gpio
// SPI 
// SPI Settings
SPIClass LoraSPI(HSPI);
SPISettings LoraSPISettings(LORAclockFrequency, MSBFIRST, SPI_MODE0);
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

    //setting CS pins to output and forcing high to make sure they are all off
    pinMode(IMUCS, OUTPUT);
    pinMode(LORACS, OUTPUT);
    digitalWrite(LORACS, HIGH);
    digitalWrite(IMUCS, HIGH);

  //start serial communication
  Serial.begin(115200);

  Serial.println("BMI270 Example 2 - Basic Readings SPI");

    // Initialize the SPI library
    SPI.begin(IMUSCK,IMUMISO,IMUMOSI, IMUCS);
    
    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
    while(imu.beginSPI(IMUCS, IMUclockFrequency) != BMI2_OK)
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
    

    // Get measurements from the sensor. This must be called before accessing
    // the sensor data, otherwise it will never update
    imu.getSensorData();
    // Get acceleration data
    AccelX = imu.data.accelX;
    Serial.println (AccelX);
    AccelY = imu.data.accelX;
    AccelZ = imu.data.accelX;

    GyroX  = imu.data.gyroX;
    GyroY  = imu.data.gyroY;
    GyroZ  = imu.data.gyroZ;

    message = String(AccelX) + String(AccelY) + String(AccelZ) + String(GyroX) + String(GyroX) + String(GyroX);
      Serial.println (message);
      DataLength = message.length();
      Serial.println (DataLength);
      Serial.println(F("[SX1262] Sending another packet ... "));
      transmissionState = radio.startTransmit(message, DataLength);
      Serial.println(digitalRead(LORACS));
      transmitFlag = true;
      Serial.println(digitalRead(LORACS));
      delay(1000);
  }

