/*
   Radio stuff
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem
   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/

   IMU stuff
   https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library
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
uint8_t LORAMOSI  = 13;
uint8_t LORAMISO  = 19;
uint8_t LORASCK   = 18;


uint32_t IMUclockFrequency = 100000;
uint8_t  IMUCS    = 5;
uint8_t IMUMOSI  = 41;
uint8_t IMUMISO  = 40; //adr
uint8_t IMUSCK   = 42;
//initialize sensor variables
float AccelX = 0;
float AccelY = 0;
float AccelZ = 0;
float GyroX  = 0;
float GyroY  = 0;
float GyroZ  = 0;

float returnFloat = 0;

//Initialize message variables// 
//This is the important stuff as this is defining my data structure

//Plays into the function, but since the max accel is 16g's, and we only get two decimal places, this is 3 bytes being the sign, integer, and decimal
uint8_t AccelXBIN;
uint8_t AccelYBIN;
uint8_t AccelZBIN;
uint8_t GyroXBIN;
uint8_t GyroYBIN;
uint8_t GyroZBIN;

uint8_t Message; 


uint8_t* result = nullptr;
int totalSize = 0;

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



//takes in the float from the sensor and makes it binary
//give it the float and then an unsigned char
void floatToBinary(const float &num, unsigned char binaryBytes[sizeof(float)])
{
        const unsigned char * numPtr = reinterpret_cast<const unsigned char *>(&num); 
        for (unsigned short i = 0; i < sizeof(float); i++)
        {
                binaryBytes[i] = (*(numPtr++)); 
        }
        return; 
}
//--------------------------------------------------------------------------------------------
//converts the Binary back to a float
void bytesToFloat(float &num, unsigned char binaryBytes[sizeof(float)])
{
        unsigned char * numPtr = reinterpret_cast<unsigned char *>(&num); 
        for (unsigned short i = 0; i < sizeof(float); i++)
        {
                (*(numPtr++)) = binaryBytes[i]; 
        }
        return; 
}
//----------------------------------------------------------------------------------------------

const int maxSize = 4; // Adjust the maximum size as needed

void concatenateArray(const uint8_t newArray[], int newArraySize, uint8_t result[], int& totalSize) {
    // Ensure we don't exceed the maximum size
    if (totalSize + newArraySize > maxSize) {
        Serial.println("Error: Exceeded maximum size");
        return;
    }

    // Copy the new array to the end of the result
    for (int i = 0; i < newArraySize; ++i) {
        result[totalSize++] = newArray[i];
    }
}


//______________________________________________________________________________________________
//
// Main Code IS Here
//
//______________________________________________________________________________________________
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
    uint8_t AccelXBIN[sizeof(AccelX)];
    floatToBinary(AccelX, AccelXBIN);
    AccelY = imu.data.accelX;
    uint8_t AccelYBIN[sizeof(AccelY)];
    floatToBinary(AccelY, AccelYBIN);
    AccelZ = imu.data.accelX;
    uint8_t AccelZBIN[sizeof(AccelZ)];
    floatToBinary(AccelZ, AccelZBIN);
    GyroX  = imu.data.gyroX;
    uint8_t GyroXBIN[sizeof(GyroX)];
    floatToBinary(GyroX, GyroXBIN);
    GyroY  = imu.data.gyroY;
    uint8_t GyroYBIN[sizeof(GyroY)];
    floatToBinary(GyroY, GyroYBIN);
    GyroZ  = imu.data.gyroZ;
    uint8_t GyroZBIN[sizeof(GyroZ)];
    floatToBinary(GyroZ, GyroZBIN);

    
    Serial.println(AccelX,6);

    bytesToFloat(returnFloat, AccelXBIN);
    Serial.println(returnFloat,6);
    
    
    uint8_t result[maxSize] = {0}; // Initialize result array
    int totalSize = 0;

    concatenateArray(AccelXBIN, sizeof(AccelXBIN), result, totalSize);
    
    Serial.println(F("[LLCC68] Sending another packet ... "));
    transmissionState = radio.startTransmit(result, sizeof(result), 0);
    transmitFlag = true;

    

    delay(1000);
  }

