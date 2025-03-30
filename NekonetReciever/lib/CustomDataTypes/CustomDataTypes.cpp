#include "CustomDataTypes.h"


// Encode a float in range [-1, 1] to 24-bit fixed-point (3 bytes)
void CustomDataTypes::encodeQuaternion(float value, uint8_t* buffer) {
    // Clamp to [-1, 1]
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;

    // Scale to signed 24-bit range
    int32_t intVal = (int32_t)(value * 8388607.0f); // 2^23 - 1

    // Store in 3 bytes (big-endian)
    buffer[0] = (intVal >> 16) & 0xFF; // MSB
    buffer[1] = (intVal >> 8) & 0xFF;
    buffer[2] = intVal & 0xFF;         // LSB
}

// Decode 3 bytes back into a float in range [-1, 1]
float CustomDataTypes::decodeQuaternion(const uint8_t* buffer) {
    // Reconstruct the signed 24-bit integer (correcting byte order)
    int32_t intVal = ((int32_t)buffer[0] << 16) | ((int32_t)buffer[1] << 8) | (int32_t)buffer[2];

    // **Fix the Sign Extension**
    if (intVal & 0x800000) { // If bit 23 is set (negative number)
        intVal -= 0x1000000; // Subtract 2^24 to properly extend the sign
    }

    // Convert back to float
    float decodedValue = (float)intVal / 8388607.0f;

    return decodedValue;
}




// Encode altitude into 3 bytes
void CustomDataTypes::encodeAltitude(float value, uint8_t* buffer) {
    // Handle the sign and get absolute value
    bool isNegative = value < 0;
    float absValue = abs(value);
    
    // Get the main part by dividing by 10
    uint16_t mainPart = (uint16_t)(absValue / 10);
    
    // Get decimal part (next two digits after division)
    float remainder = absValue - (mainPart * 10.0f);
    int8_t decimalValue = (int8_t)round(remainder * 10);  // First decimal digit
    
    // Store the values in the buffer (big-endian format)
    buffer[0] = (mainPart >> 8) & 0xFF;  // High byte
    buffer[1] = mainPart & 0xFF;         // Low byte
    buffer[2] = isNegative ? -decimalValue : decimalValue;  // Store as signed int8_
}


// Decode altitude from 3 bytes
float CustomDataTypes::decodeAltitude(const uint8_t* buffer) {
    // Extract the main part
    uint16_t mainPart = ((uint16_t)buffer[0] << 8) | buffer[1];
    
    // Extract the decimal part and sign
    int8_t decimalValue = (int8_t)buffer[2];  // Read as signed integer
    bool isNegative = decimalValue < 0;
    decimalValue = abs(decimalValue);
    
    // Reconstruct: (mainPart * 10) + (decimalPart / 10)
    float result = (mainPart * 10.0f) + (decimalValue / 10.0f);
    
    return isNegative ? -result : result;
}




void CustomDataTypes::encodeLatLon(float latitude, float longitude, uint8_t buffer[6]) {
  // Scale to maximize precision
  int32_t scaledLat = (int32_t)(latitude * 93206.73);
  int32_t scaledLon = (int32_t)(longitude * 46603.37);

  // Pack latitude (big-endian)
  buffer[0] = (scaledLat >> 16) & 0xFF;
  buffer[1] = (scaledLat >> 8) & 0xFF;
  buffer[2] = scaledLat & 0xFF;

  // Pack longitude (big-endian)
  buffer[3] = (scaledLon >> 16) & 0xFF;
  buffer[4] = (scaledLon >> 8) & 0xFF;
  buffer[5] = scaledLon & 0xFF;
}

void CustomDataTypes::decodeLatLon(uint8_t buffer[6], float &latitude, float &longitude) {
  // Unpack latitude (big-endian)
  int32_t scaledLat = ((int32_t)buffer[0] << 16) | ((int32_t)buffer[1] << 8) | buffer[2];
  if (scaledLat & 0x800000) scaledLat |= 0xFF000000; // Sign extension

  // Unpack longitude (big-endian)
  int32_t scaledLon = ((int32_t)buffer[3] << 16) | ((int32_t)buffer[4] << 8) | buffer[5];
  if (scaledLon & 0x800000) scaledLon |= 0xFF000000; // Sign extension

  // Convert back to float
  latitude = scaledLat / 93206.73f;
  longitude = scaledLon / 46603.37f;
}




void CustomDataTypes::encodePressure(float pressure, uint8_t buffer[3]) {
  // Ensure positive pressure only (no negative values allowed)
  if (pressure < 0) pressure = 0;
  
  // Scale pressure by 1000 to store three decimal places
  uint32_t scaledPressure = (uint32_t)(pressure * 1000);
  
  // Store in big-endian format
  buffer[0] = (scaledPressure >> 16) & 0xFF;
  buffer[1] = (scaledPressure >> 8) & 0xFF;
  buffer[2] = scaledPressure & 0xFF;
}

float CustomDataTypes::decodePressure(uint8_t buffer[3]) {
  // Reconstruct from big-endian format
  uint32_t scaledPressure = ((uint32_t)buffer[0] << 16) |
                            ((uint32_t)buffer[1] << 8)  |
                            (uint32_t)buffer[2];
  
  // Scale back to float
  return scaledPressure / 1000.0f;
}


void CustomDataTypes::encodeAcceleration(float acceleration, uint8_t buffer[2]) {
  // Clamp acceleration to valid range
  if (acceleration > 300.0f) acceleration = 300.0f;
  if (acceleration < -300.0f) acceleration = -300.0f;

  // Scale by 100 for two decimal places (fits within int16_t range)
  int16_t scaledAccel = (int16_t)(acceleration * 100.0f);

  // Store in big-endian format
  buffer[0] = (scaledAccel >> 8) & 0xFF;
  buffer[1] = scaledAccel & 0xFF;
}

float CustomDataTypes::decodeAcceleration(uint8_t buffer[2]) {
  // Reconstruct from big-endian format
  int16_t scaledAccel = (int16_t)((buffer[0] << 8) | buffer[1]);

  // Scale back to float
  return scaledAccel / 100.0f;
}
