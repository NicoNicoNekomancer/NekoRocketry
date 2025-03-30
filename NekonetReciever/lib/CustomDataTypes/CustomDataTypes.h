#ifndef CustomDataTypes_h
#define CustomDataTypes_h

#include <Arduino.h>
#include <math.h>


class CustomDataTypes {
  public:
      // Static functions for encoding and decoding various data types
      static void encodeQuaternion(float value, uint8_t* buffer);
      static float decodeQuaternion(const uint8_t* buffer);
  
      static void encodeAltitude(float value, uint8_t* buffer);
      static float decodeAltitude(const uint8_t* buffer);
  
      static void encodeLatLon(float latitude, float longitude, uint8_t buffer[6]);
      static void decodeLatLon(uint8_t buffer[6], float &latitude, float &longitude);
  
      static void encodePressure(float pressure, uint8_t buffer[3]);
      static float decodePressure(uint8_t buffer[3]);
  
      static void encodeAcceleration(float acceleration, uint8_t buffer[2]);
      static float decodeAcceleration(uint8_t buffer[2]);


  private:


};

#endif