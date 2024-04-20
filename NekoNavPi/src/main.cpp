#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include "IMU.h"
#include "Barometer.h"
#include "Magnetometer.h"
#include <math.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
void updateOrientation();
void calibration();
void initaialaccel();

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
// Define filter coefficients
const float alpha = 0.5;       // Coefficient for gyro data
const float beta = 1 - alpha;  // Coefficient for accelerometer data
const float alpha_gyro = 0.98; // Coefficient for gyro high-pass filter

// Define pin assignments
const int BarometerCS = 10;
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

const int IMUCS = 11;
const int MagnetometerCS = 12;
const int charges1 = 0;
const int charges2 = 1;

float temperature, pressure, altitude, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp, angleX, angleY, angleZ;
float initialAccelX, initialAccelY, initialAccelZ, initialAltitude, initialGyroX, initialGyroY, initialGyroZ, initialImutemp;
float magX, magY, magZ, magYaw;
bool AccelZNegative = true;
bool AccelXNegative = true;

float t1, t2, dt;
float accelroll, accelpitch, gyroroll, gyropitch, gyroyaw;
float roll, pitch, yaw, altitudeSlope;
float gyrorollOld = 0;
float gyropitchOld = 0;
float gyroyawOld = 0;
int gyroCalSamples = 100;
float gyroXcal, gyroYcal, gyroZcal, gyroXcal2, gyroYcal2, gyroZcal2;

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
int state = 100;
enum ProgramState
{
  READ,
  WRITE,
  DELETE,
  DEFAULT
};
ProgramState currentprogramstate = DEFAULT;

void setup()
{
  Serial.begin(115200);

  pixels.begin();
  pixels.clear();
  delay(5000);
  // Initialize the Barometer
  pinMode(charges2, OUTPUT);
  pinMode(charges1, OUTPUT);

  barometer.begin();
  imu.begin();
  magnetometer.begin();
  barometer.readData(temperature, pressure);
  magnetometer.readData(magX, magY, magZ);
  // have to read twice. assuming its some timing thing but i honestly dont know. this just works and its in the initlization so its not like it matters
  imu.readData(initialAccelX, initialAccelY, initialAccelZ, initialGyroX, initialGyroY, initialGyroZ, initialImutemp);
  imu.readData(initialAccelX, initialAccelY, initialAccelZ, initialGyroX, initialGyroY, initialGyroZ, initialImutemp);
  initialAltitude = 44330 * (1 - pow((pressure / 1013.25), 0.1903));

  calibration();
  initaialaccel();

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
      t1 = micros();
      dt = (t1 - t2) / 1000000.0;

      pixels.setPixelColor(0, pixels.Color(random(200), random(200), random(200)));
      pixels.show();
      magnetometer.readData(magX, magY, magZ);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      pixels.setPixelColor(0, pixels.Color(255, 20, 20));
      pixels.show();
      barometer.readData(temperature, pressure);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      pixels.setPixelColor(0, pixels.Color(255, 20, 20));
      pixels.show();
      imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      altitude = (44330 * (1 - pow((pressure / 1013.25), 0.1903))) - initialAltitude;
      updateOrientation();
      dataPad[0] = roll;
      dataPad[1] = pitch;
      dataPad[2] = yaw;
      for (int j = 0; j < ValuesPerPad; j++)
      {
        dataPadStorage[i][j] = dataPad[j];
      }
      dataPadStorage[PadSamples - 1][0] = i;
      t2 = t1;
      Serial.print("Roll ");
      Serial.print(roll, 4);
      Serial.print("\tPitch ");
      Serial.print(pitch, 4);
      Serial.print("\tYaw ");
      Serial.println(yaw, 4);

      delay(500);
    }

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
    while (Serial.available() == 0) {
        ; // Do nothing until data is available
    }
    // Now that data is available, read the incoming bytes
    incomingByte = Serial.read();
    byteindex = 0;

    while (incomingByte != '\n' && Serial.available() > 0 && byteindex < sizeof(receivedByte)) {
        receivedByte[byteindex] = incomingByte;
        incomingByte = Serial.read();
        byteindex += 1;
    }
    // Add the last read byte to the string
    receivedByte[byteindex] = incomingByte;

    memcpy(&callsign, &receivedByte[0], 6);
    callsign[6] = '\0';  // Null-terminate the string
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

void updateOrientation()
{

  accelroll = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
  if (accelZ < 0 && accelY > 0)
  {
    accelroll = 180 - accelroll;
  }
  else if (accelZ < 0 && accelY < 0)
  {
    accelroll = 180 - accelroll;
  }
  else if (accelZ > 0 && accelY < 0)
  {
    accelroll = 360 + accelroll;
  }

  accelpitch = -1 * atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;
  if (accelZ < 0 && accelX > 0)
  {
    accelpitch = 180 - accelpitch;
  }
  else if (accelZ < 0 && accelX < 0)
  {
    accelpitch = 180 - accelpitch;
  }
  else if (accelZ > 0 && accelX < 0)
  {
    accelpitch = 360 + accelpitch;
  }

  gyroroll = (gyrorollOld + ((gyroX - gyroXcal) * dt));
  if (gyroroll > 360)
  {
    gyroroll -= 360;
  }
  else if (gyroroll < 0)
  {
    gyroroll += 360;
  }

  gyropitch = (gyropitchOld + (-1 * (gyroY - gyroYcal) * dt));
  if (gyropitch > 360)
  {
    gyropitch -= 360;
  }
  else if (gyropitch < 0)
  {
    gyropitch += 360;
  }

  gyroyaw = (gyroyawOld + (-1 * (gyroZ - gyroZcal) * dt));
  if (gyroyaw > 360)
  {
    gyroyaw -= 360;
  }
  else if (gyroyaw < 0)
  {
    gyroyaw += 360;
  }

  magYaw = atan2(magY, magX) * 180 / PI;
  if (magX < 0 && magY > 0)
  {
    magYaw = 180 - magYaw;
  }
  else if (magX < 0 && magY < 0)
  {
    magYaw = 180 - magYaw;
  }
  else if (magX > 0 && magY < 0)
  {
    magYaw = 360 + magYaw;
  }

  if (gyroroll - accelroll > 180 || gyroroll - accelroll < -180)
  {
    accelroll = 360 - accelroll;
  }

  if (gyropitch - accelpitch > 180 || gyropitch - accelpitch < -180)
  {
    accelpitch = 360 - accelpitch;
  }
  roll = alpha * gyroroll + beta * accelroll;
  pitch = alpha * gyropitch + beta * accelpitch;
  yaw = alpha * gyroyaw + beta * magYaw;

  gyrorollOld = roll;
  gyropitchOld = pitch;
  gyroyawOld = yaw;
}

void calibration()
{
  for (int i = 0; i < gyroCalSamples; i++)
  {
    imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
    gyroXcal2 += gyroX;
    gyroYcal2 += gyroY;
    gyroZcal2 += gyroZ;
  }
  gyroXcal2 /= gyroCalSamples;
  gyroYcal2 /= gyroCalSamples;
  gyroZcal2 /= gyroCalSamples;

  for (int i = 0; i < gyroCalSamples; i++)
  {
    imu.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
    gyroXcal += gyroX - gyroXcal2;
    gyroYcal += gyroY - gyroYcal2;
    gyroZcal += gyroZ - gyroZcal2;
  }
  gyroXcal /= gyroCalSamples;
  gyroYcal /= gyroCalSamples;
  gyroZcal /= gyroCalSamples;

  gyroXcal += gyroXcal2;
  gyroYcal += gyroYcal2;
  gyroZcal += gyroZcal2;
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
