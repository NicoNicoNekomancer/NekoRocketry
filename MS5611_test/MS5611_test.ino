//
//    FILE: MS5611_test.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo application
//     URL: https://github.com/RobTillaart/MS5611_SPI


#include "MS5611_SPI.h"

//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77


//  SPI      UNO/NANO    ESP32 (V)  ESP32(H)
//  SELECT    10            5         15        CSO 
//  MOSI      11            23        13        SDI
//  MISO      12            19        12        SDO
//  CLOCK     13            18        14        SCL


// MS5611_SPI(select, dataOut, dataIn, clock);
// --------------------------------------------


// MS5611_SPI MS5611(10, 11, 12, 13);   // UNO SW SPI (5V problem?
// MS5611_SPI MS5611(10);               // UNO  HW SPI

MS5611_SPI MS5611( 6, 13, 19, 18);   // ESP32 SW SPI
//MS5611_SPI MS5611( 5, 13, 12, 14); // ESP32 HW SPI


uint32_t start, stop;


void setup()
{
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

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
    while (1)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  Serial.println();
}


/*
  There are 5 oversampling settings, each corresponding to a different amount of milliseconds
  The higher the oversampling, the more accurate the reading will be, however the longer it will take.
  OSR_ULTRA_HIGH -> 8.22 millis
  OSR_HIGH       -> 4.11 millis
  OSR_STANDARD   -> 2.1 millis
  OSR_LOW        -> 1.1 millis
  OSR_ULTRA_LOW  -> 0.5 millis   Default = backwards compatible
*/
void loop()
{

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(OSR_ULTRA_LOW);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(OSR_LOW);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(OSR_STANDARD);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(OSR_HIGH);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  digitalWrite(LED_BUILTIN, HIGH);
  MS5611.setOversampling(OSR_ULTRA_HIGH);
  test();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  Serial.println();
}


void test()
{
  start = micros();
  int result = MS5611.read();
  stop = micros();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
  }
  else
  {
    Serial.print("T:\t");
    Serial.print(MS5611.getTemperature(), 2);
    Serial.print("\tP:\t");
    Serial.print(MS5611.getPressure(), 2);
    Serial.print("\tt:\t");
    Serial.print(stop - start);
    Serial.println();
  }
}


// -- END OF FILE --

