/*
   RadioLib SX126x Ping-Pong Example

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
#include <SPI.h>


//set variables that are to be recieved
uint8_t AccelXBIN;
uint8_t AccelYBIN;
uint8_t AccelZBIN;
uint8_t GyroXBIN;
uint8_t GyroYBIN;
uint8_t GyroZBIN;

uint8_t Message; 

/******************************************************************************************************************************************************
HEY YOU
The order of this is super important as this is the packet structure. You dont want to mess around with this because it can literally break everything. 
So when the data is combined on the altimeter size, that order MUST be the same on this size when you take it apart into each individual part again. 
The general layout, past the radio stuff because we honestly dont care about what the radio does, is the following
- Callsign
  We want to make sure that the fcc at least can somewhat figure out who is sending what, plus this helps with interference issues if someone around you 
  is using another altimeter is on the same frequency. Can also allow for multi altimeter data reading by logging this but thats something for later
*/

//since these will be turned into their initial data type, also set those here
float AccelX = 0;
float AccelY = 0;
float AccelZ = 0;
float GyroX  = 0;
float GyroY  = 0;
float GyroZ  = 0;

//lets also get the size of these because we sort of need that
unsigned char AccelXSize[sizeof(AccelX)];
unsigned char AccelYSize[sizeof(AccelY)];
unsigned char AccelZSize[sizeof(AccelZ)];
unsigned char GyroXSize[sizeof(GyroX)];
unsigned char GyroYSize[sizeof(GyroY)];
unsigned char GyroZSize[sizeof(GyroZ)];

// uncomment the following only on one
// of the nodes to initiate the pings
#define INITIATING_NODE
//setting pins being used
int LORACS    = 6; //shared between spi and lora
int LORAIRQ   = 2;
int LORARST   = 3;
int LORABUSY  = 4;
int LORAMOSI  = 13;
int LORAMISO  = 19;
int LORASCK   = 18;

// SX1262 has the following connections:
// NSS pin:   cs
// DIO1 pin:  irq
// NRST pin:  rst
// BUSY pin:  gpio
// SPI 
// SPI Settings
SPIClass LoraSPI(HSPI);
SPISettings LoraSPISettings(2000000, MSBFIRST, SPI_MODE0);
LLCC68 radio = new Module(LORACS, LORAIRQ, LORARST, LORABUSY, LoraSPI, LoraSPISettings); // this one must be used because esp32



// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // we sent or received a packet, set the flag
  operationDone = true;
}





void bytesToFloat(float &num, unsigned char binaryBytes[sizeof(float)])
{
        unsigned char * numPtr = reinterpret_cast<unsigned char *>(&num); 
        for (unsigned short i = 0; i < sizeof(float); i++)
        {
                (*(numPtr++)) = binaryBytes[i]; 
        }
        return; 
}


void setup() {

  Serial.begin(115200);
  //(SCK, MISO, MOSI, CS)
  LoraSPI.begin(LORASCK,LORAMISO,LORAMOSI,LORACS);
  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));


  // carrier frequency:           Carrier frequency in MHz. Defaults to 434.0 MHz. The .0 is important because otherwise it freaks out and goes back to default
  // bandwidth:                   LoRa bandwidth in kHz. Defaults to 125.0 kHz. Up to 500 kHz. Will need to revisit datasheet to get the exact numbers
  // spreading factor:            LoRa spreading factor. Defaults to 9. {5-11}
  // coding rate:                 LoRa coding rate denominator. Defaults to 7 (coding rate 4/7) {5-8}
  // sync word:                   1-byte LoRa sync word. Defaults to RADIOLIB_SX126X_SYNC_WORD_PRIVATE (0x12). 0x34 (public network/LoRaWAN). Will add more here as I continue to learn how this works
  // output power:                Output power in dBm. Defaults to 10 dBm. Can go up to 22
  // preamble length:             LoRa preamble length in symbols. Defaults to 8 symbols.
  //tcxoVoltage	                  TCXO reference voltage to be set on DIO3. Defaults to 1.6 V. If you are seeing -706/-707 error codes, it likely means you are using non-0 value for module with XTAL. To use XTAL, either set this value to 0, or set SX126x::XTAL to true.
  //useRegulatorLDO	              Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.
  int state = radio.begin(420.0, 500.0, 11, 5, 0x34, 20, 8, 1.6, false);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  radio.setDio1Action(setFlag);

  #if defined(INITIATING_NODE)
    // send the first packet on this node
    Serial.print(F("[SX1262] Sending first packet ... "));
    transmissionState = radio.startTransmit("Hello World!");
    transmitFlag = true;
  #else
    // start listening for LoRa packets on this node
    Serial.print(F("[SX1262] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true);
    }
  #endif
}

void loop() {
  // check if the previous operation finished
  if(operationDone) {
    // reset flag
    operationDone = false;

    if(transmitFlag) {
      // the previous operation was transmission, listen for response
      // print the result
      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        Serial.println(F("transmission finished!"));

      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);

      }

      // listen for response
      radio.startReceive();
      transmitFlag = false;

    } else {
      // the previous operation was reception
      // print data and send another packet
      uint8_t data[8];
      //radio.getPacketLength();
      int state = radio.readData(data,sizeof(data));

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1262] Received packet!"));

        // print data of the packet
        Serial.print(F("[SX1262] Data:\t\t"));

          bytesToFloat(AccelX, data);
          Serial.println(AccelX,6);

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[SX1262] RSSI:\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[SX1262] SNR:\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));

      }
      transmitFlag = true;
    }
  
  }
}
