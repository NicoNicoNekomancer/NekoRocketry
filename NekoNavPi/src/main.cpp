#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include "IMU.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include "MadgwickFilter.h"
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

////////////////////////////////////////////////
//        IMPORTANT ALTIMETER CONSTANTS       //
// This value will be in meters and will be converted if its in feet. conversion will happen in the main program
const float MainAlt = 1500;
const float DrogueDelay = 0; // seconds
const float MainDelay = 0;   // seconds
const float Aux1Delay = 0;   // seconds
const float Aux2Delay = 0;   // seconds

// important altimeter constants that will be changed per the user eventually
#define AccelTHRESHOLD 0.9 // Threshold value for state transition
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

const float frequency = 420.0;
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

float t1, t2, dt, t0, tbaro;
float accelroll, accelpitch, gyroroll, gyropitch, gyroyaw;
float roll, pitch, yaw, altitudeSlope;
float gyrorollOld = 0;
float gyropitchOld = 0;
float gyroyawOld = 0;
int gyroCalSamples = 200;

float gyroXcal, gyroYcal, gyroZcal, gyroXcal2, gyroYcal2, gyroZcal2;
float accelXcal, accelYcal, accelZcal, accelXcal2, accelYcal2, accelZcal2;
float magXcal, magYcal, magZcal, magXcal2, magYcal2, magZcal2;

// gyro calibration terms
float bias_x, bias_y, bias_z;
float max_x, min_x, max_y, min_y, max_z, min_z, magCalX, magCalY, magCalZ;
// temporary values. technically accurate enough to use for hard iron, but this will be calibrated externally and stored in flash eventually
//to do a fresh calibration, values must be the following
//{0.0, 0.0, 0.0}
//{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}
float magOffset[3] = {36.500000, 321.000000, 898.000000};
float magSoftIron[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

// full scale values. these are set in the libraries so you need to change here if you change there
int accelFS = 8;
int gyroFS = 2000;
int magFS = 16;
float deg2rad = 0.0174532925;
float rad2deg = 57.2957795131;
// storage of data while on pad. will only take up so much space and replace data as it goes
const int PadSamples = 51; // treat as size 50. last one will store the current location in the array
const int ValuesPerPad = 3;
float dataPad[ValuesPerPad];
float dataPadStorage[PadSamples][ValuesPerPad];

// Create an instance of the classes

IMU imu(IMUCS, IMUMOSI, IMUMISO, IMUSCK, IMUClock);

Barometer barometer(BarometerCS, BarometerMOSI, BarometerMISO, BarometerSCK, BarometerClock);

Magnetometer magnetometer(MagnetometerCS, MagnetometerMOSI, MagnetometerMISO, MagnetometerSCK, MagnetometerClock);

// Define states
enum RocketState
{
  PROGRAM, // green
  PAD,     // red
  BOOST,   // redorange
  GLIDE,   // orange
  DROGUE,  // yellow
  MAIN,    // purple
  LANDED   // blue
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

// Define the bytes for each state
const int readState = 153;
const int writeState = 231;
const int deleteState = 52;
const int calibrateState = 262;
int state = 100;
enum ProgramState
{
  READ,
  WRITE,
  DELETE,
  CALIBRATE,
  DEFAULT
};
ProgramState currentprogramstate = DEFAULT;

// main altimeter code
void setup()
{
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
}

// radio code
void setup1()
{
  pixels.begin();
  pixels.clear();
  Serial.begin(115200);
  delay(5000); // standardizing the delay

  // start SPI1 for the radio. SPI0 is for all other sensors. although SPI1 can still be used, its best to keep it on its own for now. Infinite CS and all that, so.
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  SPI1.begin();
  // start the rest of the radio stuff

  int state = radio.begin(frequency, bandwidth, spreadingFactor, codeRate, syncWord, power, preamble, voltage, regulatorLDO);
  /*
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
  */
  radio.implicitHeader(16); // size of payload if this is always known be it payload, crc, and coding rate
  // radio.setCRC(2, 0x1D0F, 0x1021, true);
  radio.setPacketSentAction(setFlag);
}

// first read is going to have some innate error that i simply cant account for. only the first read though so its not a big deal
void loop()
{
  pixels.clear();
  switch (currentState)
  {
  case PAD:

    for (int i = 0; i < PadSamples - 1; i++)
    {
      // starts the D1 calculation
      // barometer.startADC(0);
      // starts tbaro stopwatch
      tbaro = micros();
      t1 = tbaro;
      // gets rough estimation of time between runs

      // collcts magnetometer data
      magnetometer.readData(magX, magY, magZ, magOffset, magSoftIron);

      // collects D1 and waits for the delay if needed
      // barometer.readData(temperature, pressure, 0, micros() - tbaro);
      // restarts tbaro stopwatch
      // tbaro = micros();
      // starts the D2 calculation
      // barometer.startADC(1);

      // collcts imu data
      imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
      // updates the orientation based on sensor values
      // send in gyro values in degrees / s, and accel and mag in whatever you want. its normailized anyways
      filter.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ, q1, q2, q3, q4);

      // collects D2 and waits for the delay if needed, then outputs temp and pressure
      // barometer.readData(temperature, pressure, 1, micros() - tbaro);
      // calculates altitude with an offset
      // altitude = (44330 * (1 - pow((pressure / 1013.25), 0.1903))) - initialAltitude;

      dataPad[0] = roll;
      dataPad[1] = pitch;
      dataPad[2] = yaw;
      for (int j = 0; j < ValuesPerPad; j++)
      {
        dataPadStorage[i][j] = dataPad[j];
      }
      dataPadStorage[PadSamples - 1][0] = i;

      /*
      Serial.print("Q1 ");
      Serial.print(q1, 6);
      Serial.print("\tQ2 ");
      Serial.print(q2, 6);
      Serial.print("\tQ3 ");
      Serial.print(q3, 6);
      Serial.print("\tQ4 ");
      Serial.println(q4, 6);
      */

      Serial.print(magX);
      Serial.print(",");
      Serial.print(magY);
      Serial.print(",");
      Serial.println(magZ);
      
    }

    // pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    // pixels.show();

    break;

  case BOOST:

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
      Serial.println("ACK");
      // Wait until data is available
      while (Serial.available() == 0)
      {
        ; // Do nothing until data is available
      }
      // Now that data is available, read the incoming bytes
      incomingByte = Serial.read();
      byteindex = 0;

      while (incomingByte != '\n' && Serial.available() > 0 && byteindex < sizeof(receivedByte))
      {
        receivedByte[byteindex] = incomingByte;
        incomingByte = Serial.read();
        byteindex += 1;
      }
      // Add the last read byte to the string
      receivedByte[byteindex] = incomingByte;

      memcpy(&callsign, &receivedByte[0], 6);
      callsign[6] = '\0'; // Null-terminate the string
      memcpy(&frequencyMhz, &receivedByte[6], 4);
      memcpy(&drogueDelay, &receivedByte[10], 2);
      memcpy(&mainDelay, &receivedByte[12], 2);
      memcpy(&mainAltitude, &receivedByte[14], 4);
      memcpy(&backupState, &receivedByte[18], 2);
      memcpy(&pressureTransducer, &receivedByte[20], 1);

      // Print the interpreted data
      Serial.println(callsign);
      Serial.println(frequencyMhz);
      Serial.println(drogueDelay);
      Serial.println(mainDelay);
      Serial.println(mainAltitude);
      Serial.println(backupState);
      Serial.println(pressureTransducer);

      pixels.setPixelColor(0, pixels.Color(random(200), random(200), random(200)));
      pixels.show();
      currentprogramstate = DEFAULT;
      delay(1000);
      break;

    case WRITE:

      Serial.println("FLASH_PAGE_SIZE = " + String(FLASH_PAGE_SIZE, DEC));
      Serial.println("FLASH_SECTOR_SIZE = " + String(FLASH_SECTOR_SIZE, DEC));
      Serial.println("FLASH_BLOCK_SIZE = " + String(FLASH_BLOCK_SIZE, DEC));
      Serial.println("PICO_FLASH_SIZE_BYTES = " + String(PICO_FLASH_SIZE_BYTES, DEC));
      Serial.println("XIP_BASE = 0x" + String(XIP_BASE, HEX));

      // Read the flash using memory-mapped addresses
      // For that we must skip over the XIP_BASE worth of RAM
      // int addr = FLASH_TARGET_OFFSET + XIP_BASE;
      // flash target offset is the actual spot im wanting to store data at. In thise case, its the very last sector, so id want to store data backwards.
      for (page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; page++)
      {
        addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
        p = (int *)addr;
        Serial.print("First four bytes of page " + String(page, DEC));
        Serial.print("( at 0x" + (String(int(p), HEX)) + ") = ");
        Serial.println(*p);
        if (*p == -1 && first_empty_page < 0)
        {
          first_empty_page = page;
          Serial.println("First empty page is #" + String(first_empty_page, DEC));
        }
      }

      if (Serial.available() >= DATA_SIZE)
      {
        // Read the data from the serial port
        for (int i = 0; i < DATA_SIZE; i++)
        {
          buf[i] = Serial.read();
        }

        if (first_empty_page < 0)
        {
          Serial.println("Full sector, erasing...");
          uint32_t ints = save_and_disable_interrupts();
          flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
          first_empty_page = 0;
          restore_interrupts(ints);
        }
        Serial.println("Writing to page #" + String(first_empty_page, DEC));
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page * FLASH_PAGE_SIZE), buf, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
      }
      currentprogramstate = DEFAULT;
      break;

    case CALIBRATE:
    {
      Serial.println("ACK");

      Serial.print("Current Offset: ");
      Serial.print("  X: ");
      Serial.print(magOffset[0], 6);
      Serial.print("  Y: ");
      Serial.print(magOffset[1], 6);
      Serial.print("  Z: ");
      Serial.println(magOffset[2], 6);
      float magCalOffset[3] = {0.0, 0.0, 0.0};
      magnetometer.storeOffset(magCalOffset[0], magCalOffset[1], magCalOffset[2]);
      delay(1000); // Wait for 1 seconds to ensure the Python code is ready to receive data
      magnetometer.SelfTest();

      // reset to the base state
      magnetometer.begin();
      Serial.print("Calibration starting in 3...");
      delay(1000); // Wait for 1 seconds
      Serial.print("2...");
      delay(1000); // Wait for 1 seconds
      Serial.print("1...");
      delay(1000); // Wait for 1 seconds
      Serial.println("Move altimeter in a figure 8 in the air");
      delay(500);

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
          Serial.print("Progress: ");
          Serial.print(i * 100 / 10000);
          Serial.println("%");
        }
        delay(1);
      }
      bias_x = (max_x + min_x) / 2.0;
      bias_y = (max_y + min_y) / 2.0;
      bias_z = (max_z + min_z) / 2.0;

      Serial.println("Magnetometer offset was calculated at (X, Y, Z) ");
      Serial.println(bias_x, 6);
      Serial.println(bias_y, 6);
      Serial.println(bias_z, 6);
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
        state = receivedString.toInt();

        // Check the state
        if (state == readState)
        {
          currentprogramstate = READ;
          Serial.println("Switched to READ state");
          return;
        }
        else if (state == writeState)
        {
          currentprogramstate = WRITE;
          Serial.println("Switched to WRITE state");
          return;
        }
        else if (state == deleteState)
        {
          currentprogramstate = DELETE;
          Serial.println("Switched to DELETE state");
          return;
        }
        else if (state == calibrateState)
        {
          currentprogramstate = CALIBRATE;
          Serial.println("Switched to Calibrate state");
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
void loop1()
{

  if (transmittedFlag)
  {
    // reset flag
    transmittedFlag = false;
    pixels.setPixelColor(0, pixels.Color(random(200), random(200), random(200)));
    pixels.show();

    floatToBytes(q1, Q1);
    floatToBytes(q2, Q2);
    floatToBytes(q3, Q3);
    floatToBytes(q4, Q4);
    memcpy(message, Q1, 4);
    memcpy(message + 4, Q2, 4);
    memcpy(message + 8, Q3, 4);
    memcpy(message + 12, Q4, 4);

    // transmissionState = radio.startTransmit(message, messageLength, 0);
    transmissionState = radio.startTransmit(message, 16);
  }
}

void calibration()
{

  magnetometer.storeOffset(magOffset[0], magOffset[1], magOffset[2]);

  for (int i = 0; i < gyroCalSamples; i++)
  {
    imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
    gyroXcal2 += gyroX;
    gyroYcal2 += gyroY;
    gyroZcal2 += gyroZ;

    accelXcal2 += accelX;
    accelYcal2 += accelY;
    accelZcal2 += accelZ;

    magXcal2 += magX;
    magYcal2 += magY;
    magZcal2 += magZ;
  }
  gyroXcal2 /= gyroCalSamples;
  gyroYcal2 /= gyroCalSamples;
  gyroZcal2 /= gyroCalSamples;

  accelXcal2 /= gyroCalSamples;
  accelYcal2 /= gyroCalSamples;
  accelZcal2 /= gyroCalSamples;

  magXcal2 /= gyroCalSamples;
  magYcal2 /= gyroCalSamples;
  magZcal2 /= gyroCalSamples;

  for (int i = 0; i < gyroCalSamples; i++)
  {
    imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
    gyroXcal += gyroX - gyroXcal2;
    gyroYcal += gyroY - gyroYcal2;
    gyroZcal += gyroZ - gyroZcal2;

    gyroXcal += gyroX - gyroXcal2;
    gyroYcal += gyroY - gyroYcal2;
    gyroZcal += gyroZ - gyroZcal2;

    gyroXcal += gyroX - gyroXcal2;
    gyroYcal += gyroY - gyroYcal2;
    gyroZcal += gyroZ - gyroZcal2;
  }
  gyroXcal /= gyroCalSamples;
  gyroYcal /= gyroCalSamples;
  gyroZcal /= gyroCalSamples;

  accelXcal /= gyroCalSamples;
  accelYcal /= gyroCalSamples;
  accelZcal /= gyroCalSamples;

  magXcal /= gyroCalSamples;
  magYcal /= gyroCalSamples;
  magZcal /= gyroCalSamples;

  gyroXcal += gyroXcal2;
  gyroYcal += gyroYcal2;
  gyroZcal += gyroZcal2;

  accelXcal += accelXcal2;
  accelYcal += accelYcal2;
  accelZcal += accelZcal2;

  magXcal += magXcal2;
  magYcal += magYcal2;
  magZcal += magZcal2;

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
