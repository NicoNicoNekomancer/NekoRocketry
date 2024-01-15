#include <RadioLib.h>
#include <SPI.h>

const int stringLength = 7;
const int floatCount = 8;
const int intCount = 3;

// Define the structure for the data packet
struct DataPacket {
  char callsign[stringLength];
  float floatData[floatCount];
  int intData[intCount];
};

DataPacket myDataPacket;
uint8_t binaryData[sizeof(myDataPacket)];

char extractedCallsign[stringLength];
float extractedFloatData[floatCount];
int extractedIntData[intCount];

int LORACS = 6;
int LORAIRQ = 2;
int LORARST = 3;
int LORABUSY = 4;
int LORAMOSI = 13;
int LORAMISO = 19;
int LORASCK = 18;

SPIClass LoraSPI(HSPI);
const SPISettings LoraSPISettings(8000000, MSBFIRST, SPI_MODE0);
LLCC68 radio = new Module(LORACS, LORAIRQ, LORARST, LORABUSY, LoraSPI, LoraSPISettings);

volatile bool operationDone = false;

// Function called when a complete packet is transmitted or received by the module
void setFlag(void) {
  operationDone = true;
}

void setup() {
  // Set CPU frequency to 240 MHz
  setCpuFrequencyMhz(240);
  
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize SPI for LoRa module
  LoraSPI.begin(LORASCK, LORAMISO, LORAMOSI, LORACS);

  // Initialize LoRa module with specific parameters
  int state = radio.begin(420.0, 500.0, 11, 5, 0x34, 20, 8, 1.6, false);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1262] Initializing success!"));
  } else {
    Serial.print(F("[SX1262] Initializing failed, code "));
    Serial.println(state);
    while (true);
  }

  // Set the function that will be called when a new packet is received
  radio.setDio1Action(setFlag);

  // Start listening for LoRa packets on this node
  state = radio.startReceive();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1262] Starting to listen success!"));
  } else {
    Serial.print(F("[SX1262] Starting to listen failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  // Check if a radio operation (transmission or reception) is complete
  if (operationDone) {
    operationDone = false;

    // Read data from the radio buffer
    int state = radio.readData(binaryData, sizeof(binaryData));

    // Check if data was successfully received
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("[SX1262] Received packet!"));

      // Decode the received binary data into the data packet struct
      decodeDataPacket();

      // Display the decoded data packet
      printDataPacket();

      // Extract values from the decoded data packet
      extractValues();
    }

    // After reception, start listening for the next packet
    radio.startReceive();
  }
}

// Function to encode data packet into binary representation
void encodeDataPacket() {
  memcpy(binaryData, &myDataPacket, sizeof(myDataPacket));
}

// Function to decode binary data into the data packet struct
void decodeDataPacket() {
  memcpy(&myDataPacket, binaryData, sizeof(myDataPacket));
}

// Function to extract values from the data packet struct
void extractValues() {
  char* callsignPtr = myDataPacket.callsign;
  float* floatDataPtr = myDataPacket.floatData;
  int* intDataPtr = myDataPacket.intData;

  Serial.println("Extracted Values:");
  Serial.print("Callsign: ");
  Serial.println(callsignPtr);

  Serial.print("Floats: ");
  for (int i = 0; i < floatCount; ++i) {
    Serial.print(floatDataPtr[i], 6);
    Serial.print(" ");
  }

  Serial.print("\nInts: ");
  for (int i = 0; i < intCount; ++i) {
    Serial.print(intDataPtr[i]);
    Serial.print(" ");
  }

  Serial.println();
}

// Function to print the entire data packet
void printDataPacket() {
  char* callsignPtr = myDataPacket.callsign;
  float* floatDataPtr = myDataPacket.floatData;
  int* intDataPtr = myDataPacket.intData;

  Serial.print("Callsign: ");
  Serial.println(callsignPtr);

  Serial.print("Floats: ");
  for (int i = 0; i < floatCount; ++i) {
    Serial.print(floatDataPtr[i], 2);
    Serial.print(" ");
  }

  Serial.print("\nInts: ");
  for (int i = 0; i < intCount; ++i) {
    Serial.print(intDataPtr[i]);
    Serial.print(" ");
  }

  Serial.println();
}
