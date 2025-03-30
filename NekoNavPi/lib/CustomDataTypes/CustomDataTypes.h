#ifndef CustomDataTypes_h
#define CustomDataTypes_h

#include <Arduino.h>
#include <math.h>


class CustomDataTypes {
public:

void encodeQuaternion(float value, uint8_t* buffer);
float decodeQuaternion(const uint8_t* buffer);

void encodeAltitude(float value, uint8_t* buffer);
float decodeAltitude(const uint8_t* buffer);

void encodeLatLon(float latitude, float longitude, uint8_t buffer[6]);
void decodeLatLon(uint8_t buffer[6], float &latitude, float &longitude);

void encodePressure(float pressure, uint8_t buffer[3]);
float decodePressure(uint8_t buffer[3]);

void encodeAcceleration(float acceleration, uint8_t buffer[2]);
float decodeAcceleration(uint8_t buffer[2]);



private:


};

#endif