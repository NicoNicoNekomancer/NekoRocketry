 /*
 Once a packet is received, an interrupt is triggered. 
 To successfully receive data, the following settings have to be the same
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word
*/
#include <RadioLib.h>
#include <SPI.h>
#include <Adafruit_TinyUSB.h>
#include "CustomDataTypes.h"



// setting pins being used
int LORACS = 3; // shared between spi and lora
int LORAIRQ = 20;
int LORARST = 15;
int LORABUSY = 2;
int LORAMOSI = 11;
int LORAMISO = 12;
int LORASCK = 10;
// SX1262 has the following connections:
// NSS pin:   cs
// DIO1 pin:  irq
// NRST pin:  rst
// BUSY pin:  gpio
// SPI
// SPI Settings
SPISettings spiSettings(18000000, MSBFIRST, SPI_MODE0); // 18 MHz clock speed
LLCC68 radio = new Module(LORACS, LORAIRQ, LORARST, LORABUSY, SPI1, spiSettings);

// carrier frequency:           Carrier frequency in MHz. Defaults to 434.0 MHz. The .0 is important because otherwise it freaks out and goes back to default
// bandwidth:                   LoRa bandwidth in kHz. Defaults to 125.0 kHz. Up to 500 kHz. Will need to revisit datasheet to get the exact numbers
// spreading factor:            LoRa spreading factor. Defaults to 9. {5-11}
// coding rate:                 LoRa coding rate denominator. Defaults to 7 (coding rate 4/7) {5-8}
// sync word:                   1-byte LoRa sync word. Defaults to RADIOLIB_SX126X_SYNC_WORD_PRIVATE (0x12). 0x34 (public network/LoRaWAN). Will add more here as I continue to learn how this works
// output power:                Output power in dBm. Defaults to 10 dBm. Can go up to 22
// preamble length:             LoRa preamble length in symbols. Defaults to 8 symbols.
// tcxoVoltage	                TCXO reference voltage to be set on DIO3. Defaults to 1.6 V. If you are seeing -706/-707 error codes, it likely means you are using non-0 value for module with XTAL. To use XTAL, either set this value to 0, or set SX126x::XTAL to true.
// useRegulatorLDO	            Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.

const float frequency = 440.0;
const float bandwidth = 500;
const uint8_t spreadingFactor = 9;
const uint8_t codeRate = 5;
const uint8_t syncWord = 0x12; // will be able to change this.
const int8_t power = 20;
const uint16_t preamble = 8;
const float voltage = 1.6;
const bool regulatorLDO = false;
const uint8_t RadioAddress = 0; // address for FSK. default is 0.

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;



/*
Accel X     2 bytes
Accel Y     2 bytes
Accel Z     2 bytes
Q0          3 bytes
Q1          3 bytes
Q2          3 bytes
Q3          3 bytes
Alt         3 bytes
dt          2 bytes
state       1 byte
{total}     {24 bytes}
addons
pressure    3 bytes
Lat         3 bytes
Long        3 bytes
{total}     {9 bytes}
*/
struct SensorData {

  uint8_t AccelX[2]; // Encoded X acceleration (2 bytes)
  uint8_t AccelY[2]; // Encoded Y acceleration (2 bytes)
  uint8_t AccelZ[2]; // Encoded Z acceleration (2 bytes)
  uint8_t Q0[3];
  uint8_t Q1[3];
  uint8_t Q2[3];
  uint8_t Q3[3];
  uint8_t Alt[3];
  uint8_t dT[2];
  uint8_t State[1];
  
};
//initiialize the sensor data ahead of time
SensorData sensordata;

CustomDataTypes customdatatypes;

// flag to indicate that a packet was sent or received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void)
{
  // we sent or received a packet, set the flag
  receivedFlag = true;
}



float floatArray[4];
float floatArrayOld[4];



void setup()
{
  TinyUSBDevice.setManufacturerDescriptor("NekoRocketry");
  TinyUSBDevice.setProductDescriptor("Nekonet Reciever");
  //vid, pid
  TinyUSBDevice.setID(0x732B, 0x8CB0); //0x5b99 for the other



  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  SPI1.begin();
  Serial.begin(115200);
  delay(5000); // needed?
  // initialize SX1262 with default settings

  // carrier frequency:           Carrier frequency in MHz. Defaults to 434.0 MHz. The .0 is important because otherwise it freaks out and goes back to default
  // bandwidth:                   LoRa bandwidth in kHz. Defaults to 125.0 kHz. Up to 500 kHz. Will need to revisit datasheet to get the exact numbers
  // spreading factor:            LoRa spreading factor. Defaults to 9. {5-11}
  // coding rate:                 LoRa coding rate denominator. Defaults to 7 (coding rate 4/7) {5-8}
  // sync word:                   1-byte LoRa sync word. Defaults to RADIOLIB_SX126X_SYNC_WORD_PRIVATE (0x12). 0x34 (public network/LoRaWAN). Will add more here as I continue to learn how this works
  // output power:                Output power in dBm. Defaults to 10 dBm. Can go up to 22
  // preamble length:             LoRa preamble length in symbols. Defaults to 8 symbols.
  // tcxoVoltage	                  TCXO reference voltage to be set on DIO3. Defaults to 1.6 V. If you are seeing -706/-707 error codes, it likely means you are using non-0 value for module with XTAL. To use XTAL, either set this value to 0, or set SX126x::XTAL to true.
  // useRegulatorLDO	              Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.
  int state = radio.begin(frequency, bandwidth, spreadingFactor, codeRate, syncWord, power, preamble, voltage, regulatorLDO);

  if (state == RADIOLIB_ERR_NONE)
  {
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  //radio.setCRC(2, 0x1D0F, 0x1021, true);
  //radio.implicitHeader(16); //size of payload if this is always known be it payload, crc, and coding rate
  radio.explicitHeader();

  radio.setPacketReceivedAction(setFlag);
  state = radio.startReceive();
  Serial.println("Starting");
}


void loop() {
  if (receivedFlag) {
      receivedFlag = false;

      SensorData receivedData;
      int state = radio.readData((uint8_t*)&receivedData, sizeof(SensorData));

      if (state == RADIOLIB_ERR_NONE) {
          Serial.print("DATA: ");

          // Print Acceleration (Decoded from 2 bytes)
          Serial.print(CustomDataTypes::decodeAcceleration(receivedData.AccelX)); Serial.print(" ");
          Serial.print(CustomDataTypes::decodeAcceleration(receivedData.AccelY)); Serial.print(" ");
          Serial.print(CustomDataTypes::decodeAcceleration(receivedData.AccelZ)); Serial.print(" ");

          // Print Quaternions (Decoded from 3 bytes)
          Serial.print(CustomDataTypes::decodeQuaternion(receivedData.Q0)); Serial.print(" ");
          Serial.print(CustomDataTypes::decodeQuaternion(receivedData.Q1)); Serial.print(" ");
          Serial.print(CustomDataTypes::decodeQuaternion(receivedData.Q2)); Serial.print(" ");
          Serial.print(CustomDataTypes::decodeQuaternion(receivedData.Q3)); Serial.print(" ");

          // Print Altitude (Decoded from 3 bytes)
          Serial.print(CustomDataTypes::decodeAltitude(receivedData.Alt)); Serial.print(" ");

          // Print dT (Time Difference, 2 bytes)
          uint16_t dT;
          memcpy(&dT, receivedData.dT, sizeof(dT));
          Serial.print(dT); Serial.print(" ");

          // Print State (1 byte)
          Serial.println(receivedData.State[0]);
      }
  }
}
