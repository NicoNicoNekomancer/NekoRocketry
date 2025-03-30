#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include "IMU.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include "MadgwickFilter.h"
#include "MahonyFilter.h"
#include "CustomDataTypes.h"
#include "FlashStorage.h"
#include <math.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <RadioLib.h>
#include <Adafruit_TinyUSB.h>

void calibration();
void initaialaccel();
void intToBytes(int val, uint8_t *bytes_array);
void longToBytes(long val, uint8_t *bytes_array);
void floatToBytes(float val, uint8_t *bytes_array);
float getVerticalAcceleration();
float calculateVariance(float *buffer, int size);

////////////////////////////////////////////////
//        IMPORTANT ALTIMETER CONSTANTS       //
// This value will be in meters and will be converted if its in feet. conversion will happen in the main program
const float MainAlt = 1500;
const float DrogueDelay = 0; // seconds
const float MainDelay = 0;   // seconds
const float Aux1Delay = 0;   // seconds
const float Aux2Delay = 0;   // seconds

// important altimeter constants that will be changed per the user eventually
#define AccelTHRESHOLD 0.8 // Threshold value for state transition
#define DETECTION_ANGLE 30  // Change this to set the tilt threshold

////////////////////////////////////////////////

////////////////////////////////////////////////
//         IMPORTANT Radio CONSTANTS          //
int LORACS = 3; // shared between spi and lora
int LORAIRQ = 20;
int LORARST = 15;
int LORABUSY = 2;
int LORAMOSI = 11;
int LORAMISO = 12;
int LORASCK = 10;
SPISettings spiSettings(16000000, MSBFIRST, SPI_MODE0); // 18 MHz clock speed
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
const int8_t power = 22;
const uint16_t preamble = 8;
const float voltage = 1.6;
const bool regulatorLDO = false;
const uint8_t RadioAddress = 0; // address for FSK. default is 0.
// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;
// flag to indicate that a packet was sent. Set as true as this sends inside of the loop
volatile bool transmittedFlag = true;

void setFlag(void)
{
  // we sent a packet, set the flag
  transmittedFlag = true;
}
////////////////////////////////////////////////
//    values that will be sent over radio     //
uint8_t Q1[4];
uint8_t Q2[4];
uint8_t Q3[4];
uint8_t Q4[4];
uint8_t message[16];
size_t messageLength = 16;
////////////////////////////////////////////////

////////////////////////////////////////////////
// Madgwick Filter
Madgwick filter;
Mahony filter2;
float q1, q2, q3, q4;
////////////////////////////////////////////////

// Define pin assignments
const int BarometerCS = 13;
const int BarometerMOSI = 19;
const int BarometerMISO = 16;
const int BarometerSCK = 18;
const int BarometerClock = 10000000;

const int IMUMOSI = 19;
const int IMUMISO = 16;
const int IMUSCK = 18;
const int IMUClock = 10000000;

const int MagnetometerMOSI = 19;
const int MagnetometerMISO = 16;
const int MagnetometerSCK = 18;
const int MagnetometerClock = 10000000;

const int IMUCS = 8;
const int MagnetometerCS = 7;
const int charges1 = 0;
const int charges2 = 1;

float temperature, pressure, altitude, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp, angleX, angleY, angleZ, g;
float initialAccelX, initialAccelY, initialAccelZ, initialAltitude, initialGyroX, initialGyroY, initialGyroZ, initialImutemp;
float magX, magY, magZ, magYaw;
bool AccelZNegative = true;
bool AccelXNegative = true;

float t1, t2, t0, tbaro;
short dt;
float accelroll, accelpitch, gyroroll, gyropitch, gyroyaw;
float roll, pitch, yaw, altitudeSlope;
float gyrorollOld = 0;
float gyropitchOld = 0;
float gyroyawOld = 0;
int gyroCalSamples = 200;

float gyroXcal, gyroYcal, gyroZcal, gyroXcal2, gyroYcal2, gyroZcal2;
float accelXcal, accelYcal, accelZcal, accelXcal2, accelYcal2, accelZcal2;
float magXcal, magYcal, magZcal, magXcal2, magYcal2, magZcal2;

//low pass filter terms
#define ALPHA 0.8 //can be between 0.7 and 0.9 for best results

float lastGyroX = 0.0f, lastGyroY = 0.0f, lastGyroZ = 0.0f, lastAccelX = 0.0f, lastAccelY = 0.0f, lastAccelZ = 0.0f, lastMagX = 0.0f, lastMagY = 0.0f, lastMagZ = 0.0f;



// gyro calibration terms
float bias_x, bias_y, bias_z;
float max_x, min_x, max_y, min_y, max_z, min_z, magCalX, magCalY, magCalZ;
// temporary values. technically accurate enough to use for hard iron, but this will be calibrated externally and stored in flash eventually
//to do a fresh calibration, values must be the following
//{0.0, 0.0, 0.0}
//{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}
float magOffset[3] = {9,2,0}; //current {9,2,0}  
float magSoftIron[3][3] = {{1.020979, 0.0, 0.0}, {0.0, 0.999240, 0.0}, {0.0, 0.0, 0.980597}}; //initial values are diag 1 or {{1.020979, 0.0, 0.0}, {0.0, 0.999240, 0.0}, {0.0, 0.0, 0.980597}}

// full scale values. these are set in the libraries so you need to change here if you change there
int accelFS = 8;
int gyroFS = 2000;
int magFS = 16;
float deg2rad = 0.0174532925;
float rad2deg = 57.2957795131;
// storage of data while on pad. will only take up so much space and replace data as it goes
const int PadSamples = 501; // treat as size x + 1. last one will store the current location in the array
const int ValuesPerPad = 8;
float dataPad[ValuesPerPad];
float dataPadStorage[PadSamples][ValuesPerPad];
//value is summed as data is read in. Its then divided by the i of the loop t get the average.
//its Y instead of Z as its vertical in this state
float sumAccelY;
float sumAlt;
float averageAccelY; 
float averageAlt;

float sumq0;
float averageq0;
float sumq1;
float averageq1;
float sumq2;
float averageq2;
float sumq3;
float averageq3;

// create the arrays to store data



// Create an instance of the classes

IMU imu(IMUCS, IMUMOSI, IMUMISO, IMUSCK, IMUClock);

Barometer barometer(BarometerCS, BarometerMOSI, BarometerMISO, BarometerSCK, BarometerClock);

Magnetometer magnetometer(MagnetometerCS, MagnetometerMOSI, MagnetometerMISO, MagnetometerSCK, MagnetometerClock);

FlashStorage flashstorage;

CustomDataTypes customdatatypes;

// Create an instance of the data that will be saved to the flash and send over radio

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

// Define states
enum RocketState
{
  PROGRAM, // white
  PAD,     // red
  BOOST,   // orange
  GLIDE,   // yellow
  DROGUE,  // green
  MAIN,    // blue
  LANDED   // purple
};
RocketState currentState = PAD;

// led stuff
#define PIN 23
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB);
#define DELAYVAL 500

// storage stuff
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
uint8_t buf[FLASH_PAGE_SIZE]; // one page buffer
int *p, addr, rlen;
unsigned int page; // prevent comparison of unsigned and signed int
int first_empty_page = -1;

// programing state stuff
#define DATA_SIZE 28
byte incomingByte;
byte receivedByte[21];
int byteindex;
char callsign[7];
float frequencyMhz;
short drogueDelay, mainDelay, backupState;
long mainAltitude;
bool pressureTransducer;

// Define the code as bytes for each state
const char readState = 153;
const char writeState = 231;
const char deleteState = 52;
const char calibrateState = 255;
char state = 100; //this is our reserved value. never have anything set to this
enum ProgramState
{
  READ,
  WRITE,
  DELETE,
  CALIBRATE,
  DEFAULT
};
ProgramState currentprogramstate = DEFAULT;


//setting the time
#define FIXED_DT 0.05f  // Define your desired fixed delta time in seconds (e.g., 0.02s = 50Hz)
unsigned long previousMillis = 0;  // Store the time of the last update
unsigned long currentMillis = 0;   // Store the current time
float dT = FIXED_DT;               // Fixed time step
const unsigned long dT_ms = dT * 1000;


// main altimeter code
void setup()
{
  pixels.begin();
  pixels.clear();
  pinMode(charges2, OUTPUT);
  pinMode(charges1, OUTPUT);

  TinyUSBDevice.setManufacturerDescriptor("NekoRocketry");
  TinyUSBDevice.setProductDescriptor("Nekonav");
  //vid, pid
  TinyUSBDevice.setID(0x5b99, 0x8CB0); 

  Serial.begin(115200);
  delay(5000); // delay required for serial monitor to print stuff. will be removed when its no longer needed

  // Initialize the Sensors
  barometer.begin();
  imu.begin();
  magnetometer.begin();

  barometer.startADC();
  barometer.readData(temperature, pressure);
  barometer.startADC();
  barometer.readData(temperature, pressure);
  magnetometer.readData(magX, magY, magZ, magOffset, magSoftIron);
  // have to read twice. assuming its some timing thing but i honestly dont know. this just works and its in the initlization so its not like it matters
  imu.readData(initialAccelX, initialAccelY, initialAccelZ, initialGyroX, initialGyroY, initialGyroZ, initialImutemp);
  imu.readData(initialAccelX, initialAccelY, initialAccelZ, initialGyroX, initialGyroY, initialGyroZ, initialImutemp);
  initialAltitude = 44330 * (1 - pow((pressure / 1013.25), 0.1903));
  // Serial.println(initialAltitude);
  calibration();
  // re reading the values. again, its the setup so i really dont care. might adjust this later.
  imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, initialImutemp);
  magnetometer.readData(magX, magY, magZ, magOffset, magSoftIron);

  initaialaccel();
  magnetometer.initializeCalibration();
  

}

// radio code
void setup1()
{

  Serial.begin(115200);
  delay(5000); // standardizing the delay

  // start SPI1 for the radio. SPI0 is for all other sensors. although SPI1 can still be used, its best to keep it on its own for now. Infinite CS and all that, so.
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  SPI1.begin();
  delay(5000); // needed?
  // start the rest of the radio stuff

  int state = radio.begin(frequency, bandwidth, spreadingFactor, codeRate, syncWord, power, preamble, voltage, regulatorLDO);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  //radio.implicitHeader(16); // size of payload if this is always known be it payload, crc, and coding rate
  radio.explicitHeader();

  // radio.setCRC(2, 0x1D0F, 0x1021, true);
  radio.setPacketSentAction(setFlag);


}

// first read is going to have some innate error that i simply cant account for. only the first read though so its not a big deal
int blink = 0;
bool haslooped = false;

int BaroLoops = 0;
//
// start of the loops are here
//        
// Initialize velocity variables
float velocity = 0.0f;



void loop()
{
   
  switch (currentState)
  {
    case PAD:
    for (int i = 0; i < PadSamples - 1; i++) {
      currentMillis = millis();
      dt = currentMillis - previousMillis;

      if (dt >= dT_ms) {
        previousMillis = currentMillis;

        Serial.print("DEBUG: ");
        Serial.println(dt);

        // Barometer state machine
        switch(BaroLoops) {
          case 0:
            // Start first conversion
            barometer.startADC();
            break;
            
          case 2:
            // Read first conversion and start second
            barometer.readData(temperature, pressure);
            barometer.startADC();
            break;
            
          case 4:
            // Read second conversion and calculate altitude
            barometer.readData(temperature, pressure);
            altitude = (44330 * (1 - pow((pressure / 1013.25), 0.1903))) - initialAltitude;
            
            // State transition check
            if (i >= 100 && averageAccelY >= 3 && averageAlt >= 3) {
              currentState = BOOST;
              break;
            }
            
            // Reset the counter
            BaroLoops = -1; // Will become 0 after increment
            break;
        }
        
        // Always increment BaroLoops
        BaroLoops++;
        
        // These can run every loop
        magnetometer.readData(magX, magY, magZ, magOffset, magSoftIron);
        //magnetometer.applyCalibration(magX, magY, magZ);
        imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
        
        accelX -= accelXcal;
        accelY -= accelYcal;
        accelZ -= accelZcal;
        gyroX -= gyroXcal;
        gyroY -= gyroYcal;
        gyroZ -= gyroZcal;
        
        filter.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ, q1, q2, q3, q4, FIXED_DT);
    

        // Store current readings
        dataPad[0] = accelX;
        dataPad[1] = accelY;
        dataPad[2] = accelZ;
        dataPad[3] = altitude;  // This will use the last valid altitude reading
        dataPad[4] = q1;
        dataPad[5] = q2;
        dataPad[6] = q3;
        dataPad[7] = q4;

        // Update rolling sums
        sumAccelY = 0;
        sumAlt = 0;
        sumq0 = 0;
        sumq1 = 0;
        sumq2 = 0;
        sumq3 = 0;
        
        // Store new data and calculate sums
        for (int j = 0; j < ValuesPerPad; j++) {
          dataPadStorage[i][j] = dataPad[j];
        }
        dataPadStorage[PadSamples - 1][0] = i;

        // Calculate sums using a window of the last 100 samples or less
        int windowSize = min(i + 1, 100);  // Use either all samples up to i, or last 100
        int startIdx = max(0, i - 99);     // Start index for rolling window
        
        for (int k = startIdx; k <= i; k++) {
          sumAccelY += dataPadStorage[k][1];
          sumAlt += dataPadStorage[k][3];
          sumq0 += dataPadStorage[k][4];
          sumq1 += dataPadStorage[k][5];
          sumq2 += dataPadStorage[k][6];
          sumq3 += dataPadStorage[k][7];
        }

        // Calculate averages based on actual window size
        averageAccelY = sumAccelY / windowSize;
        averageAlt = sumAlt / windowSize;
        averageq0 = sumq0 / windowSize;
        averageq1 = sumq1 / windowSize;
        averageq2 = sumq2 / windowSize;
        averageq3 = sumq3 / windowSize;

        /* 
        Serial.print("DEBUG: ");
        Serial.print(q1);
        Serial.print(", ");
        Serial.print(q2);
        Serial.print(", ");
        Serial.print(q3);
        Serial.print(", ");
        Serial.print(q4);
        Serial.print(", ");
        Serial.print(dt);
        Serial.print(", ");
        Serial.println(averageAlt);
        */
       
        /*  THIS IS THE OUTPUT FOR THE MANGETOMETER CALIBRATION!!!!!!!
        Serial.print("LOG: ");
        Serial.print(magX);
        Serial.print(",");
        Serial.print(magY);
        Serial.print(",");
        Serial.println(magZ);
        */


        
        float verticalAccel = getVerticalAcceleration();
        //X/Z plane tilt. absolute pita to get this to work though. 
        // Compute the "up" vector from the quaternion
        float up_x = 2 * (q2 * q4 - q1 * q3);
        float up_y = 2 * (q3 * q4 + q1 * q2);
        float up_z = 1 - 2 * (q2 * q2 + q3 * q3);  // Z-component

        // Compute magnitude of the up vector
        float up_magnitude = sqrt(up_x * up_x + up_y * up_y + up_z * up_z);

        // Compute TOTAL TILT (X/Z plane tilt)
        float total_tilt = acos(sqrt(up_x * up_x + up_z * up_z) / up_magnitude) * (180.0 / M_PI);

        // Check if tilt is inside the threshold
        bool tilt_inside = total_tilt > DETECTION_ANGLE;

        // Print results
        //Serial.print("Total Tilt: ");
        //Serial.print(total_tilt);
        //Serial.print(" degrees, Tilt Exceeded: ");
        Serial.println(tilt_inside ? "TRUE" : "FALSE");



      }
    }

    // LED blinking logic
    if (blink == 1000) {
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
    }
    if (blink == 2000) {
      pixels.setPixelColor(0, pixels.Color(255, 10, 10));
      pixels.show();
      blink = 0;
    }
    blink++;
    
    break;

  case BOOST:

    pixels.setPixelColor(0, pixels.Color(255, 150, 10)); //orange
    pixels.show();
    Serial.print("LOG:");
    Serial.println(dataPadStorage[PadSamples - 1][0]);
    delay(1000);
    break;

  case GLIDE:

    break;

  case DROGUE:

    break;

  case MAIN:

    break;

  case LANDED:

    break;

  case PROGRAM: /*
                Program state AKA horizontal. Will take in serial input to program.
                Values will be Drogue Delay, Main Altitude, Main Delay, Drogue Terminal, Main Terminal, Alt Terminal 1, Alt Terminal 2, and then a Bool for if the Alts are Backups
                Will need to get the values stored in the NVM. Should be able to use shorts for most of hte data with one being a long.
                readState = 153; READ
                writeState = 231; WRITE
                deleteState = 52; DELETE
                and then DEFAULT
                */
    switch (currentprogramstate)
    {

    case READ:
      {
        Serial.println("LOG:ACK");  // Send ACK immediately


        pixels.setPixelColor(0, pixels.Color(200, 100, 50));
        pixels.show();

        currentprogramstate = DEFAULT;
        break;
      }
    case WRITE:


      currentprogramstate = DEFAULT;
      break;

    case CALIBRATE:
    {
      Serial.println("LOG:ACK");
    
      Serial.print("LOG:Current Offset: ");
      Serial.print("  X: ");
      Serial.print(magOffset[0], 6);
      Serial.print("  Y: ");
      Serial.print(magOffset[1], 6);
      Serial.print("  Z: ");
      Serial.println(magOffset[2], 6);
    
      float magCalOffset[3] = {0.0, 0.0, 0.0};
      float magSoftIron[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
      //magnetometer.storeOffset(magCalOffset[0], magCalOffset[1], magCalOffset[2]);
      delay(1000);
      magnetometer.SelfTest();
      magnetometer.begin();
    
      Serial.println("LOG:Calibration starting in ");
      Serial.println("LOG:3...");
      delay(1000);
      Serial.println("LOG:2...");
      delay(1000);
      Serial.println("LOG:1...");
      delay(1000);
      Serial.println("LOG:Move altimeter in a figure 8 in the air");
      delay(500);
    
      float max_x = -3.4e38, min_x = 3.4e38;
      float max_y = -3.4e38, min_y = 3.4e38;
      float max_z = -3.4e38, min_z = 3.4e38;
      
    
      for (int i = 0; i < 10000; i++)
      {
        magnetometer.readData(magCalX, magCalY, magCalZ, magCalOffset, magSoftIron);
    
        max_x = max(max_x, magCalX);
        min_x = min(min_x, magCalX);
        max_y = max(max_y, magCalY);
        min_y = min(min_y, magCalY);
        max_z = max(max_z, magCalZ);
        min_z = min(min_z, magCalZ);
    
        if (i % (10000 / 10) == 0)
        {
          Serial.print("LOG:Progress: ");
          Serial.print(i * 100 / 10000);
          Serial.println("%");
        }
        delay(1);
      }
    
      bias_x = (max_x + min_x) / 2.0;
      bias_y = (max_y + min_y) / 2.0;
      bias_z = (max_z + min_z) / 2.0;
    
      float scale_x = (max_x - min_x) / 2.0;
      float scale_y = (max_y - min_y) / 2.0;
      float scale_z = (max_z - min_z) / 2.0;
      float avg_scale = (scale_x + scale_y + scale_z) / 3.0;
      
      magSoftIron[0][0] = avg_scale / scale_x;
      magSoftIron[1][1] = avg_scale / scale_y;
      magSoftIron[2][2] = avg_scale / scale_z;
    
      Serial.println("LOG:Magnetometer offset was calculated at (X, Y, Z) ");
      Serial.print("LOG:"); Serial.println(bias_x, 6);
      Serial.print("LOG:"); Serial.println(bias_y, 6);
      Serial.print("LOG:"); Serial.println(bias_z, 6);
      
      Serial.println("LOG:Soft iron correction matrix:");
      Serial.print("LOG:"); Serial.print(magSoftIron[0][0], 6); Serial.print(", "); Serial.print(magSoftIron[0][1], 6); Serial.print(", "); Serial.println(magSoftIron[0][2], 6);
      Serial.print("LOG:"); Serial.print(magSoftIron[1][0], 6); Serial.print(", "); Serial.print(magSoftIron[1][1], 6); Serial.print(", "); Serial.println(magSoftIron[1][2], 6);
      Serial.print("LOG:"); Serial.print(magSoftIron[2][0], 6); Serial.print(", "); Serial.print(magSoftIron[2][1], 6); Serial.print(", "); Serial.println(magSoftIron[2][2], 6);
    
      currentprogramstate = DEFAULT;
      break;
    }
      
    case DEFAULT:

      for (int i = 0; i < 255; i++)
      {
        pixels.setPixelColor(0, pixels.Color(i, i, i));
        pixels.show();
        delay(10);
      }
      if (Serial.available() > 0)
      {
        String receivedString = Serial.readStringUntil('\n');
        state = (char)receivedString.toInt();

        // Check the state
        if (state == readState)
        {
          currentprogramstate = READ;
          Serial.println("LOG:Switched to READ state");
          return;
        }
        else if (state == writeState)
        {
          currentprogramstate = WRITE;
          Serial.println("LOG:Switched to WRITE state");
          return;
        }
        else if (state == deleteState)
        {
          currentprogramstate = DELETE;
          Serial.println("LOG:Switched to DELETE state");
          return;
        }
        else if (state == calibrateState)
        {
          currentprogramstate = CALIBRATE;
          Serial.println("LOG:Switched to Calibrate state");
          return;
        }
       
      }

      for (int i = 255; i > 0; i--)
      {
        pixels.setPixelColor(0, pixels.Color(i, i, i));
        pixels.show();
        delay(10);
      }
      break;
    }
  }
}


// radio loop. just sends data. thats it. Might also use it to save but not sure
void loop1() {
  if (transmittedFlag) {
      transmittedFlag = false;

      SensorData sensordata;

      // Encode sensor values before transmission
      customdatatypes.encodeAcceleration(accelX, sensordata.AccelX);
      customdatatypes.encodeAcceleration(accelY, sensordata.AccelY);
      customdatatypes.encodeAcceleration(accelZ, sensordata.AccelZ);
      customdatatypes.encodeQuaternion(q1, sensordata.Q0);
      customdatatypes.encodeQuaternion(q2, sensordata.Q1);
      customdatatypes.encodeQuaternion(q3, sensordata.Q2);
      customdatatypes.encodeQuaternion(q4, sensordata.Q3);
      customdatatypes.encodeAltitude(altitude, sensordata.Alt);
      memcpy(sensordata.dT, &dt, sizeof(dt));
      sensordata.State[0] = static_cast<uint8_t>(currentState);

      // Send struct as raw bytes
      transmissionState = radio.startTransmit((uint8_t*)&sensordata, sizeof(SensorData));
      //Serial.println(transmissionState);
      delay(5);
  }
}

void calibration() {
  int samples = 100;
  Serial.println("Starting IMU & Magnetometer Calibration...");
  Serial.println("Keep device stationary during calibration...");
  
  // Initialize sums for averaging
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float magXMin = 10000, magYMin = 10000, magZMin = 10000;
  float magXMax = -10000, magYMax = -10000, magZMax = -10000;
  
  // Step 1: Collect sensor data
  for (int i = 0; i < samples; i++) {
      imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
      magnetometer.readData(magX, magY, magZ, magOffset, magSoftIron);

      // Accumulate gyroscope readings
      gyroXSum += gyroX;
      gyroYSum += gyroY;
      gyroZSum += gyroZ;

      // Accumulate accelerometer readings
      accelXSum += accelX;
      accelYSum += accelY;
      accelZSum += accelZ;

      // Find magnetometer min/max for hard-iron calibration
      if (magX < magXMin) magXMin = magX;
      if (magY < magYMin) magYMin = magY;
      if (magZ < magZMin) magZMin = magZ;
      if (magX > magXMax) magXMax = magX;
      if (magY > magYMax) magYMax = magY;
      if (magZ > magZMax) magZMax = magZ;

      delay(5);
  }

  // Step 2: Compute gyroscope biases (offsets) - this is correct
  gyroXcal = gyroXSum / samples;
  gyroYcal = gyroYSum / samples;
  gyroZcal = gyroZSum / samples;

  // Step 3: Compute accelerometer biases properly
  // Calculate average readings
  float accelXAvg = accelXSum / samples;
  float accelYAvg = accelYSum / samples;
  float accelZAvg = accelZSum / samples;
  
  // Calculate magnitude of the acceleration vector
  float accelMagnitude = sqrt(accelXAvg*accelXAvg + accelYAvg*accelYAvg + accelZAvg*accelZAvg);
  
  // Normalize to gravity (should be close to 1g or 9.81 m/s²)
  float scaleFactor = 1.0 / accelMagnitude;
  
  // Calculate bias as deviation from expected normalized values
  // We're assuming any static position, so we preserve the gravity direction
  // but normalize its magnitude
  accelXcal = accelXAvg - (accelXAvg * scaleFactor);
  accelYcal = accelYAvg - (accelYAvg * scaleFactor);
  accelZcal = accelZAvg - (accelZAvg * scaleFactor);

  // Step 4: Compute magnetometer hard-iron offset (center of min/max)
  magXcal = (magXMax + magXMin) / 2.0f;
  magYcal = (magYMax + magYMin) / 2.0f;
  magZcal = (magZMax + magZMin) / 2.0f;

  Serial.println("Calibration Complete!");
  Serial.print("Gyro Bias: ");
  Serial.print(gyroXcal); Serial.print(", ");
  Serial.print(gyroYcal); Serial.print(", ");
  Serial.println(gyroZcal);

  Serial.print("Accel Bias: ");
  Serial.print(accelXcal); Serial.print(", ");
  Serial.print(accelYcal); Serial.print(", ");
  Serial.println(accelZcal);
  
  Serial.print("Accel Scale Factor: ");
  Serial.println(scaleFactor);

  Serial.print("Magnetometer Offset: ");
  Serial.print(magXcal); Serial.print(", ");
  Serial.print(magYcal); Serial.print(", ");
  Serial.println(magZcal);
}


void initaialaccel()
{

  // Check if accelZ is positive. Setting to .1 to try and account for any strange behavior
  if (initialAccelZ > 0)
  {
    AccelZNegative = false;
    // Check if accelZ is greater than threshold
    if (initialAccelZ > AccelTHRESHOLD)
    {
      currentState = PROGRAM; // Transition to PROGRAM state
    }
    else
    {
      currentState = PAD; // Transition to PAD state
    }
  }
  // Check if accelZ is negative
  else if (initialAccelZ < 0)
  {
    // Check if accelZ is less than negative threshold
    if (initialAccelZ < -AccelTHRESHOLD)
    {
      currentState = PROGRAM; // Transition to PROGRAM state
    }
    else
    {
      currentState = PAD; // Transition to PAD state
    }
  }
  /*
  if (initialAccelY > 0){
    if (initialAccelZ > AccelTHRESHOLD){
      imu.remapAxisSign(1);
    }
  }
  */
  // this reads in the starting positing from the accelerometer, as this should in most cases be the propper reference.
  // will eventually update this to instead work off of sensor fusion with the magnetometer but thats a thing for later
  accelroll = atan(initialAccelY / sqrt(pow(initialAccelX, 2) + pow(initialAccelZ, 2))) * 180 / PI;
  if (initialAccelZ < 0 && initialAccelY > 0)
  {
    accelroll = 180 - accelroll;
  }
  else if (initialAccelZ < 0 && initialAccelY < 0)
  {
    accelroll = 180 - accelroll;
  }
  else if (initialAccelZ > 0 && initialAccelY < 0)
  {
    accelroll += 360 + accelroll;
  }

  accelpitch = -1 * atan(-1 * initialAccelX / sqrt(pow(initialAccelY, 2) + pow(initialAccelZ, 2))) * 180 / PI;
  if (initialAccelZ < 0 && initialAccelX > 0)
  {
    accelpitch = 180 - accelpitch;
  }
  else if (initialAccelZ < 0 && initialAccelX < 0)
  {
    accelpitch = 180 - accelpitch;
  }
  else if (initialAccelZ > 0 && initialAccelX < 0)
  {
    accelpitch = 360 + accelpitch;
  }

  gyrorollOld = accelroll;
  gyropitchOld = accelpitch;
}

void floatToBytes(float val, uint8_t *bytes_array)
{
  union
  {
    float float_variable;
    uint8_t temp_array[4];
  } u;

  u.float_variable = val;

  // Assign bytes to the array
  bytes_array[0] = u.temp_array[0];
  bytes_array[1] = u.temp_array[1];
  bytes_array[2] = u.temp_array[2];
  bytes_array[3] = u.temp_array[3];
}
void longToBytes(long val, uint8_t *bytes_array)
{
  union
  {
    long long_variable;
    uint8_t temp_array[4];
  } u;

  u.long_variable = val;

  // Assign bytes to the array
  bytes_array[0] = u.temp_array[0];
  bytes_array[1] = u.temp_array[1];
  bytes_array[2] = u.temp_array[2];
  bytes_array[3] = u.temp_array[3];
  /* example
  long myLong = 1234567890L;
  uint8_t bytes[4];
  longToBytes(myLong, bytes);
  */
}
void intToBytes(int val, uint8_t *bytes_array)
{
  union
  {
    int int_variable;
    uint8_t temp_array[2];
  } u;

  u.int_variable = val;

  // Assign bytes to the array
  bytes_array[0] = u.temp_array[0];
  bytes_array[1] = u.temp_array[1];
}


const float deadbandThreshold = 0.02f; // Deadband threshold to reduce drift
bool stabilized = false;     // Stabilization flag
unsigned long stabilizationStartTime = 0; // Timer for stabilization
const unsigned long stabilizationTime = 3000; // 3 seconds stabilization time
const float stabilizationVarianceThreshold = 0.005f; // Variance threshold for stabilization
const int stabilizationSamples = 100; // Number of samples to check
float accelBuffer[stabilizationSamples];
int sampleIndex = 0;
bool varianceCheckPassed = false;

float calculateVariance(float *buffer, int size) {
  float mean = 0.0f;
  for (int i = 0; i < size; i++) {
    mean += buffer[i];
  }
  mean /= size;

  float variance = 0.0f;
  for (int i = 0; i < size; i++) {
    variance += (buffer[i] - mean) * (buffer[i] - mean);
  }
  return variance / size;
}

float getVerticalAcceleration() {
  struct Quaternion {
    float w, x, y, z;
  };

  // Quaternion multiplication
  auto quatMultiply = [](Quaternion q1, Quaternion q2) -> Quaternion {
    return {q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w};
  };

  // Conjugate of quaternion
  auto quatConjugate = [](Quaternion q) -> Quaternion {
    return {q.w, -q.x, -q.y, -q.z};
  };

  Quaternion q = {q1, q2, q3, q4};
  Quaternion v = {0.0f, accelX, accelY, accelZ};
  Quaternion qConj = quatConjugate(q);
  Quaternion result = quatMultiply(quatMultiply(q, v), qConj);

  // Gravity compensation (assuming 1.0 g along Earth Z-axis)
  float verticalAccel = result.z - 1.0f;

  // Apply deadband to vertical acceleration
  if (fabs(verticalAccel) < deadbandThreshold) {
    verticalAccel = 0.0f;
  }

  // Stabilization routine with variance check
  if (!stabilized) {
    accelBuffer[sampleIndex] = verticalAccel;
    sampleIndex = (sampleIndex + 1) % stabilizationSamples;

    if (sampleIndex == 0) {
      float variance = calculateVariance(accelBuffer, stabilizationSamples);
      if (variance < stabilizationVarianceThreshold) {
        varianceCheckPassed = true;
      }
    }

    if (varianceCheckPassed && (millis() - stabilizationStartTime >= stabilizationTime)) {
      stabilized = true;
      velocity = 0.0f; // Reset velocity after stabilization
    }

    if (stabilizationStartTime == 0) {
      stabilizationStartTime = millis();
    }

    return 0.0f; // No velocity integration during stabilization
  }

  // Velocity integration using global dt in milliseconds
  if (dt > 0) {
    velocity += verticalAccel * (dt / 1000.0f); // Convert ms to seconds
  }

  return verticalAccel;
}
