/* 
helper to save data to the flash chip. There are a few considerations that MUST be done and things to do prior to using this which is both setting a filesystem up
as well as sending the size of the filesystem. Since it indexes from the end we use the start location as where all of the data is stored. IF this location changes, 
there is no real way to make things work fine so just dont do that
functions should be called in a specific order as well

example code for saving stuff to flash memory
https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/ see here for explinations for some of it

You can write in pages taking up the whole page and then delete in sectors, which cotains the pages
So for example, this test code would be 

LOG:FLASH_PAGE_SIZE = 256 //The size of one page, in bytes (the mimimum amount you can write)
LOG:FLASH_SECTOR_SIZE = 4096 //The size of one sector, in bytes (the minimum amount you can erase)
LOG:FLASH_BLOCK_SIZE = 65536
LOG:PICO_FLASH_SIZE_BYTES = 4194304 //The total size of the RP2040 flash, in bytes
LOG:XIP_BASE = 0x10000000 //The location where program data begins

Now, two main fuctions exist
flash_range_erase(uint32_t flash_offs, size_t count);
flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);
the first lets you erase, the second lest you write. 

  Read the flash using memory-mapped addresses
  For that we must skip over the XIP_BASE worth of RAM
  int addr = FLASH_TARGET_OFFSET + XIP_BASE;
  We also want to start with the first page in the sector, so page index of 0 * flash_page_size indexes at this start. 

*/

#ifndef FlashStorage_h
#define FlashStorage_h

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <hardware/flash.h>
#include <hardware/sync.h>

struct ConfigData {
  float frequency;          // 4 bytes
  uint16_t main_altitude;   // 2 bytes
  char callsign[6];         // 6 bytes
  uint8_t primary_delay;    // 1 byte
  uint8_t secondary_delay;  // 1 byte
  uint8_t backup_state;     // 1 byte
  bool pressure_transducer; // 1 byte
};

class FlashStorage {
public:
   
    //sets the config data
    void performConfigFlashOperations(const ConfigData& config);

    void forceConfigWipe();
    void forceFlightWipe();

    void readConfigData();

    void printConfigData(const ConfigData *data);

    int  findNextEmptyPageWithErase();

    void writeToFlash(const uint8_t* data, size_t dataSize);

    void eraseSector(int sectorIndex);

    void readPage(int sectorIndex, int pageIndex);

    bool checkSector(int sectorIndex);

    void addToBuffer(const void* data, size_t dataSize);

    void scanFlashForRegions();

    void viewSensorDataRegion(int regionIndex);

    void requestUserRegionSelection();
    //erases a region. Send in the start addr and end addr of the region you want to delete. 
    void eraseFlashRegion(uint32_t start_addr, uint32_t end_addr);

    void flipNextBit();
    bool readCurrentBit();

    
private:
      // Global variables to store the empty page information

      uint32_t g_firstEmptyPageAddr = 0xFFFFFFFF; // Default to an invalid address
      int g_firstEmptyPageIndex = -1;
      int g_firstEmptySectorIndex = -1;
      int g_pageWithinSector = -1;
      int g_sectorInRangeIndex = -1;
      //struct to store Sensor Data Regions IE where data has been saved from flights. Plus some other stuff
      struct SensorDataRegion {
          int startPage;
          int startSector;
          int endPage;
          int endSector;
          int regionID;
      };
      #define MAX_REGIONS 50  // Adjust as needed
      SensorDataRegion regions[MAX_REGIONS];
      int regionCount = 0;
      #define FILESYSTEM_SIZE_BYTES (2 * 1024 * 1024) // 2.00 MB filesystem
      // Configuration data location in flash memory
      // Config data starts after the filesystem
      #define CONFIG_DATA (FILESYSTEM_SIZE_BYTES)    
      //Flight Log Sector
      #define FlightSector (CONFIG_DATA + FLASH_SECTOR_SIZE) 
      // Sensor data starts after config sector
      #define SensorSector (FlightSector + FLASH_SECTOR_SIZE) 
      #define FileSystemEnd (PICO_FLASH_SIZE_BYTES)

      // Define constants
      #define SUBPAGE_SIZE 16 // Each subpage is 16 bytes
      #define SUBPAGES_PER_PAGE (FLASH_PAGE_SIZE / SUBPAGE_SIZE) // 256 / 16 = 16 subpages per page
      #define SECTOR_PAGE_COUNT (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)

      // Must match flash page size
      #define FLASH_BUFFER_SIZE FLASH_PAGE_SIZE  
      // Buffer to hold data before writing
      uint8_t flashBuffer[FLASH_BUFFER_SIZE];  
      // Tracks position in buffer
      int bufferIndex = 0; 

      // Buffer to store binary data
      uint8_t Config_buf[FLASH_PAGE_SIZE];  // Full page buffer (256 bytes)
      int *p;
      unsigned int page, subpage;
      int addr;
      int first_empty_page = -1;
      int first_empty_subpage = -1;

      // Buffer to hold incoming sensor data (256 bytes per flash page)
      uint8_t Sensor_buf[FLASH_PAGE_SIZE];
      unsigned int sensor_buf_index = 0;   // Tracks the current position in the buffer
      unsigned int current_page = 0;       // Tracks the current flash page being written

      // Variable to track the current sector dynamically
      uint32_t current_sector = SensorSector;


      // Flag to track first write of session
      bool firstWrite = true;  
      // 4-byte start indicator
      const uint32_t START_MARKER = 0x4E454B4F;
          


      //int for the current flight
      int FlightNumber = 0;
};

#endif