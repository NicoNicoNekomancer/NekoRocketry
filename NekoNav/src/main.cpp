#include <Arduino.h>
#include "Barometer.h"
#include "IMU.h"
#include "Magnetometer.h"
#include <math.h>
void updateOrientation();
void calibration();
void initaialaccel();


////////////////////////////////////////////////
//        IMPORTANT ALTIMETER CONSTANTS       //
//This value will be in meters and will be converted if its in feet. conversion will happen in the main program
const float MainAlt = 1500;
const float DrogueDelay = 0; //seconds
const float MainDelay = 0;   //seconds
const float Aux1Delay = 0;   //seconds
const float Aux2Delay = 0;   //seconds



//important altimeter constants that will be changed per the user eventually
#define AccelTHRESHOLD 0.9 // Threshold value for state transition
// Define filter coefficients
const float alpha = 0.95; // Coefficient for gyro data
const float beta = 1 - alpha;  // Coefficient for accelerometer data
const float alpha_gyro = 0.98; // Coefficient for gyro high-pass filter



// Define pin assignments
const int BarometerCS    = 7;
const int BarometerMOSI  = 41;
const int BarometerMISO  = 40;
const int BarometerSCK   = 42;
const int BarometerClock = 10000000;
const int IMUCS = 5;
const int MagnetometerCS = 15;
const int charges1 = 47;
const int charges2 = 48;


float temperature, pressure, altitude, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp, angleX, angleY, angleZ;
float initialAccelX, initialAccelY, initialAccelZ, initialAltitude, initialGyroX, initialGyroY, initialGyroZ, initialImutemp;
float MagnetometerX, MagnetometerY, MagnetometerZ;
bool AccelZNegative = true;
bool AccelXNegative = true;
 

float t1, t2, dt;
float accelroll, accelpitch, gyroroll, gyropitch, gyroyaw;  
float roll, pitch, yaw, altitudeSlope;
float gyrorollOld    = 0;
float gyropitchOld   = 0;
float gyroyawOld     = 0;
int   gyroCalSamples = 100;
float gyroXcal, gyroYcal, gyroZcal, gyroXcal2, gyroYcal2, gyroZcal2;


//storage of data while on pad. will only take up so much space and replace data as it goes
const int PadSamples = 51; //treat as size 50. last one will store the current location in the array
const int ValuesPerPad = 3;
float dataPad[ValuesPerPad];
float dataPadStorage[PadSamples][ValuesPerPad];


// Create an instance of the classes
Barometer barometer(BarometerCS, BarometerMOSI, BarometerMISO, BarometerSCK, BarometerClock);
IMU IMU(IMUCS, BarometerMOSI, BarometerMISO, BarometerSCK, BarometerClock);
Magnetometer Magnetometer(MagnetometerCS, BarometerMOSI, BarometerMISO, BarometerSCK, BarometerClock);

// Define states
enum RocketState {
  UNARMED,
  PAD,
  BOOST,
  GLIDE, 
  DROGUE,
  MAIN,
  LANDED
};
RocketState currentState = UNARMED; 

void setup() {
  Serial.begin(115200);
  // Initialize the Barometer
  pinMode(charges2, OUTPUT);
  pinMode(charges1, OUTPUT);


  barometer.begin();
  IMU.begin();
  Magnetometer.begin();
  delay(2000);
  barometer.readData(temperature, pressure);
  Magnetometer.readData(MagnetometerX, MagnetometerY, MagnetometerZ);
  //have to read twice. assuming its some timing thing but i honestly dont know. this just works and its in the initlization so its not like it matters
  IMU.readData(initialAccelX, initialAccelY, initialAccelZ, initialGyroX, initialGyroY, initialGyroZ, initialImutemp);
  IMU.readData(initialAccelX, initialAccelY, initialAccelZ, initialGyroX, initialGyroY, initialGyroZ, initialImutemp);
  initialAltitude = 44330 * (1 - pow((pressure / 1013.25), 0.1903));

  calibration();
  initaialaccel();

  Serial.println(currentState);

  
}


//first read is going to have some innate error that i simply cant account for. only the first read though so its not a big deal
void loop() {
  switch(currentState) {
  case PAD:
    for(int i = 0; i < PadSamples - 1; i++){
      t1 = micros();
      dt = (t1-t2) / 1000000.0;
      Magnetometer.readData(MagnetometerX, MagnetometerY, MagnetometerZ);
      barometer.readData(temperature, pressure);
      IMU.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
      altitude = (44330 * (1 - pow((pressure / 1013.25), 0.1903))) - initialAltitude;
      updateOrientation();
      dataPad[0] = roll;
      dataPad[1] = pitch;
      dataPad[2] = yaw;
      for (int j = 0; j < ValuesPerPad; j++) {
        dataPadStorage[i][j] = dataPad[j];
      }
      dataPadStorage[PadSamples-1][0] = i;
      t2 = t1;
      Serial.print("Roll ");
      Serial.print(accelroll,4);
      Serial.print("\tPitch ");
      Serial.print(accelpitch,4);
      Serial.print("\tYaw ");
      Serial.print(yaw,4);
      Serial.print("\tRoll ");
      Serial.print(accelX,4);
      Serial.print("\tPitch ");
      Serial.print(accelY,4);
      Serial.print("\tYaw ");
      Serial.print(accelZ,4);
      Serial.print("\tMagX ");
      Serial.print(MagnetometerX,4);
      Serial.print("\tMagY ");
      Serial.print(MagnetometerY,4);
      Serial.print("\tMagZ ");
      Serial.println(MagnetometerZ,4);
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

  }
}



void updateOrientation() {
  
// Calculate pitch angle


  accelroll = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
  if(accelZ < 0 && accelY > 0){
    accelroll = 180 - accelroll;
  }else if(accelZ < 0 && accelY < 0 ){
    accelroll = 180 - accelroll;
  }else if(accelZ > 0 && accelY < 0){
    accelroll = 360 + accelroll;
  }

  accelpitch = -1 * atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;
  if(accelZ < 0 && accelX > 0){
    accelpitch = 180 - accelpitch;
  }else if(accelZ < 0 && accelX < 0){
    accelpitch = 180 - accelpitch;
  }else if(accelZ > 0 && accelX < 0){
    accelpitch = 360 + accelpitch;
  }

  gyroroll = (gyrorollOld + ((gyroX - gyroXcal) * dt));
  if(gyroroll > 360){
    gyroroll -= 360;
  }else if(gyroroll < 0){
    gyroroll += 360;
  }

  gyropitch = (gyropitchOld + (-1 * (gyroY -  gyroYcal) * dt));
  if(gyropitch > 360){
    gyropitch -= 360;
  }else if(gyropitch < 0){
    gyropitch += 360;
  }

  gyroyaw = (gyroyawOld +  (-1 * (gyroZ - gyroZcal) * dt));
  if(gyroyaw  > 360){
    gyroyaw  -= 360;
  }else if(gyroyaw < 0){
    gyroyaw  += 360;
  }

if(gyroroll - accelroll > 180 || gyroroll - accelroll < -180){
    accelroll = 360 - accelroll;
  }

if(gyropitch - accelpitch > 180 || gyropitch - accelpitch < -180){
    accelpitch = 360 - accelpitch;
  }
  roll = alpha * gyroroll + beta * accelroll;
  pitch = alpha * gyropitch + beta * accelpitch;
  yaw = gyroyaw;

  gyrorollOld = roll;
  gyropitchOld = pitch;
  gyroyawOld = yaw;
}

void calibration() {
  for(int i = 0; i < gyroCalSamples; i++){
    IMU.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
    gyroXcal2 += gyroX;
    gyroYcal2 += gyroY;
    gyroZcal2 += gyroZ;
  }
  gyroXcal2 /= gyroCalSamples;
  gyroYcal2 /= gyroCalSamples;
  gyroZcal2 /= gyroCalSamples; 

  for(int i = 0; i < gyroCalSamples; i++){
    IMU.readData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, imutemp);
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


void initaialaccel(){

    // Check if accelZ is positive. Setting to .1 to try and account for any strange behavior
  if (initialAccelZ > 0) {
    AccelZNegative = false;
    // Check if accelZ is greater than threshold
    if (initialAccelZ > AccelTHRESHOLD) {
      currentState = UNARMED; // Transition to UNARMED state
    }else {
    currentState = PAD; // Transition to PAD state
  }
  }
  // Check if accelZ is negative
  else if (initialAccelZ < 0) {
    // Check if accelZ is less than negative threshold
    if (initialAccelZ < -AccelTHRESHOLD) {
      currentState = UNARMED; // Transition to UNARMED state
    }else {
    currentState = PAD; // Transition to PAD state
  }
  }

  // this reads in the starting positing from the accelerometer, as this should in most cases be the propper reference. 
  //will eventually update this to instead work off of sensor fusion with the magnetometer but thats a thing for later
   accelroll = atan(initialAccelY / sqrt(pow(initialAccelX, 2) + pow(initialAccelZ, 2))) * 180 / PI;
  if(initialAccelZ < 0 && initialAccelY > 0){
    accelroll = 180 - accelroll;
  }else if(initialAccelZ < 0 && initialAccelY < 0){
    accelroll = 180 - accelroll;
  }else if(initialAccelZ > 0 && initialAccelY < 0){
    accelroll += 360 + accelroll;
  }

  accelpitch = -1 * atan(-1 * initialAccelX / sqrt(pow(initialAccelY, 2) + pow(initialAccelZ, 2))) * 180 / PI;
  if(initialAccelZ < 0 && initialAccelX > 0){
    accelpitch = 180 - accelpitch;
  }else if(initialAccelZ < 0 && initialAccelX < 0){
    accelpitch = 180 - accelpitch;
  }else if(initialAccelZ > 0 && initialAccelX < 0){
    accelpitch = 360 + accelpitch;
  }

  gyrorollOld = accelroll;
  gyropitchOld = accelpitch;
}






