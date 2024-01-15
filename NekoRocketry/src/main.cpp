/* !! Important Info About The Chips And All That !!
   Radio stuff
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem
   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/

   IMU stuff
   https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library

   Barometer Stuff AUTHOR: Rob Tillaart
   https://github.com/RobTillaart/MS5611_SPI
*/

// include the library
#include <SPI.h>
#include <RadioLib.h> //Radio LLCC68
#include "SparkFun_BMI270_Arduino_Library.h" //IMU
#include "MS5611_SPI.h" //Barometer

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

//IMU STUFF IMU STUFF IMU STUFF IMU STUFF
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

/*BAROMETER STUFF BAROMETER STUFF BAROMETER STUFF
  BREAKOUT  MS5611  aka  GY63 - see datasheet

  SPI    I2C
              +--------+
  VCC    VCC  | o      |
  GND    GND  | o      |
         SCL  | o      |
  SDI    SDA  | o      |
  CSO         | o      |
  SDO         | o L    |   L = led
          PS  | o    O |   O = opening  PS = protocol select
              +--------+

  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
  PS to GND  ==>  SPI
  CS to VCC  ==>  0x76
  CS to GND  ==>  0x77


  Pin Connections For SPI. This is shared for all sensors.
      
  SELECT / CSO    7
  MOSI   / SDI    41
  MISO   / SDO    40
  CLOCK  / SCL    42

*/
uint8_t BarometerCS    = 7;
uint8_t BarometerMOSI  = 21;// 41
uint8_t BarometerMISO  = 47; // 40
uint8_t BarometerSCK   = 17;// 42
// CS, MOSI, MISO, SCK
MS5611_SPI MS5611(BarometerCS, BarometerMOSI, BarometerMISO, BarometerSCK);
// not sure what this is, lol
uint32_t start, stop;


// Initialize message variables// 
// THIS IS THE IMPORTANT STUFF FOR THE DATA PACKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// Define the number of elements in the data packet. Since I know what I'll have, I can set it all here. 
// Will flesh out as I go, but the idea is to start with the Callsign, then the IMU, then Barometer, then GPS, and then whatever else I decide to add in

/* This is the CallSign including null terminator. 6 characters is default and then the null. 
Will have to adjust this to allow for someone with a shorter callsign */
const int stringLength = 7; 

/* This is for the sensors mainly. 6 come from the IMU, 2 come from the Barometer, */
const int floatCount = 8;

/* This is just for testing and isnt important at all. wanting to test data types*/
const int intCount = 3;

// Define the struct for the data packet
struct DataPacket {
  char callsign[stringLength];
  float floatData[floatCount];
  int intData[intCount];
};

// Create an instance of the struct
DataPacket myDataPacket;

uint8_t binaryData[sizeof(myDataPacket)];

// Variables to store extracted values
char extractedCallsign[stringLength];
float extractedFloatData[floatCount];
int extractedIntData[intCount];


//end of important stuff !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


//Plays into the function
uint8_t AccelXBIN;
uint8_t AccelYBIN;
uint8_t AccelZBIN;
uint8_t GyroXBIN;
uint8_t GyroYBIN;
uint8_t GyroZBIN;

uint8_t Message; 


uint8_t* result = nullptr;
int totalSize = 0;
/*
LLCC68 has the following connections:
NSS pin:   cs
DIO1 pin:  irq
NRST pin:  rst
BUSY pin:  gpio

Look at the datasheet to figure out what the pins are, lol.

SPI Settings
*/
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



//--------------------------------------------------------------------------------------------
//Encodes the packet as a binary array. 
void encodeDataPacket() {
  // Copy the struct to the binary buffer
  memcpy(binaryData, &myDataPacket, sizeof(myDataPacket));
}
//--------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------






//______________________________________________________________________________________________
//
// Main Code IS Here
//
//______________________________________________________________________________________________
void setup() {
  //setting callsign here. Will eventually be some other function but idk how to do that yet, lol
  //final version will NOT function this way asside from it being a callsign and being in the setup
  strcpy(myDataPacket.callsign, "KF0HVS");

  //setting random ints for testing reasons. just want to make sure that this can be used for different data types. 
  //will probs just be a bunch of floats in the final code, but whatever
  myDataPacket.intData[0] = 123;
  myDataPacket.intData[1] = 456;
  myDataPacket.intData[2] = 789;

    //setting CS pins to output and forcing high to make sure they are all off
    pinMode(IMUCS, OUTPUT);
    pinMode(LORACS, OUTPUT);
    digitalWrite(LORACS, HIGH);
    digitalWrite(IMUCS, HIGH);

  //start serial communication
  Serial.begin(115200);

  //IMU
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

//Barometer
  while(!Serial);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("MS5611_SPI_LIB_VERSION: ");
  Serial.println(MS5611_SPI_LIB_VERSION);
  if (MS5611.begin() == true)
  {
    Serial.println("MS5611 found.");
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    while (1);
  }
  Serial.println();


}
void loop() {
    

  // Get measurements from the sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu.getSensorData();
    // Get acceleration data in g's
    //AccelX 
    myDataPacket.floatData[0] = imu.data.accelX;

    Serial.println(myDataPacket.floatData[0],6);
    //AccelY 
    myDataPacket.floatData[1] = imu.data.accelY;
    
    //AccelZ 
    myDataPacket.floatData[2] = imu.data.accelZ;

    // Get gyro data in rad/s 
    //GyroX  
    myDataPacket.floatData[3] = imu.data.gyroX;
    
    //GyroY  
    myDataPacket.floatData[4] = imu.data.gyroY;
    
    //GyroZ  
    myDataPacket.floatData[5] = imu.data.gyroZ;
    
  

    int BarometerResult = MS5611.read();   // uses default OSR_ULTRA_LOW  (fastest)
      
    //BarometerTemperature 
    myDataPacket.floatData[6] = MS5611.getTemperature();
    myDataPacket.floatData[7] = MS5611.getPressure();

    Serial.println(myDataPacket.floatData[7],6);

    //encodes the packet as binary. Stored in binaryData
    encodeDataPacket();
    
    Serial.println(sizeof(myDataPacket));
    for(int i = 0; i < floatCount; i++){
        Serial.print(myDataPacket.floatData[i]);
        Serial.print(", ");
        }
        Serial.println("");
    for(int i = 0; i < sizeof(binaryData); i++){
        Serial.print(binaryData[i]);
        }
   Serial.println("End Of Binary");
    
    
    Serial.println(F("[LLCC68] Sending another packet ... "));
    transmissionState = radio.startTransmit(binaryData, sizeof(binaryData), 0);
    transmitFlag = true;

    

    delay(10);
  }

