#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// Create a new sensor object
BMI270 imu;

// SPI parameters
uint8_t chipSelectPin = 5;
uint32_t clockFrequency = 100000;
int IMUCS     = 5; //shared between spi and lora
int LORACS     = 6;
int LORAIRQ   = 2;
int LORARST   = 3;
int LORABUSY  = 4;
int LORAMOSI  = 13;
int LORAMISO  = 19;
int LORASCK   = 18;


void setup()
{
    // Start serial
    pinMode(IMUCS, OUTPUT);
    pinMode(LORACS, OUTPUT);
    digitalWrite(LORACS, HIGH);

    Serial.begin(115200);
    Serial.println("BMI270 Example 2 - Basic Readings SPI");

    // Initialize the SPI library
    SPI.begin(LORASCK,LORAMISO,LORAMOSI, IMUCS);

    
    imu.beginSPI(IMUCS, clockFrequency);
    
    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
    /*while(imu.beginSPI(IMUCS, clockFrequency) != BMI2_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMI270 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }*/

    Serial.println("BMI270 connected!");
}

void loop()
{
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

    // Print 50x per second
    delay(2000);
}