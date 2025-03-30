#include "FlashStorage.h"


// Flash operations specific to ConfigData
void FlashStorage::performConfigFlashOperations(const ConfigData& config) {
    Serial.println("Testing Config Data Flash Binary Read/Write");

    // Initialize buffer to erased state (0xFF)
    memset(Config_buf, 0xFF, sizeof(Config_buf));

    // Find the first empty subpage
    first_empty_page = -1;
    first_empty_subpage = -1;

    for (page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; page++) {
        addr = XIP_BASE + CONFIG_DATA + (page * FLASH_PAGE_SIZE);
        for (subpage = 0; subpage < SUBPAGES_PER_PAGE; subpage++) {
            uint32_t subpage_addr = addr + (subpage * SUBPAGE_SIZE);
            bool is_empty = true;

            // Check if subpage is empty
            for (int i = 0; i < SUBPAGE_SIZE; i++) {
                if (*(uint8_t *)(subpage_addr + i) != 0xFF) {
                    is_empty = false;
                    break;
                }
            }

            if (is_empty) {
                first_empty_page = page;
                first_empty_subpage = subpage;
                break;
            }
        }
        if (first_empty_page >= 0) break;
    }

    if (first_empty_page < 0 || first_empty_subpage < 0) {
        Serial.println("Sector is full, erasing...");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(CONFIG_DATA, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        first_empty_page = 0;
        first_empty_subpage = 0;
        Serial.println("Sector erased. Starting from page #0, subpage #0.");
    }

    // Copy the passed-in struct into the buffer
    memcpy(Config_buf, &config, sizeof(ConfigData));

    // Write the entire ConfigData struct to the specific subpage
    Serial.println("Writing configuration to page #" + String(first_empty_page) +
                   ", subpage #" + String(first_empty_subpage));

    uint32_t flash_addr = CONFIG_DATA + (first_empty_page * FLASH_PAGE_SIZE) +
                          (first_empty_subpage * SUBPAGE_SIZE);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(flash_addr, Config_buf, sizeof(ConfigData));
    restore_interrupts(ints);

    Serial.println("Configuration data written successfully.");
}





// Function to forcefully wipe the Config sector
void FlashStorage::forceConfigWipe() {
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(CONFIG_DATA, FLASH_SECTOR_SIZE);
  restore_interrupts(ints);
  Serial.println("Flash sector successfully erased.");
}

// Function to forcefully wipe the Flight sector
void FlashStorage::forceFlightWipe() {
  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FlightSector, FLASH_SECTOR_SIZE);
  restore_interrupts(ints);
  Serial.println("Flash sector successfully erased.");
}



// Function to read the last written configuration data
void FlashStorage::readConfigData() {
  int last_written_page = -1;
  int last_written_subpage = -1;

  // Scan the entire configuration sector in reverse to find the last written subpage
  for (int page = (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE) - 1; page >= 0; page--) {
    uint32_t page_addr = CONFIG_DATA + (page * FLASH_PAGE_SIZE);

    for (int subpage = SUBPAGES_PER_PAGE - 1; subpage >= 0; subpage--) {
      uint32_t subpage_addr = page_addr + (subpage * SUBPAGE_SIZE);
      bool is_written = false;

      // Check if this subpage contains any data other than 0xFF
      for (int i = 0; i < SUBPAGE_SIZE; i++) {
        if (*(uint8_t *)(XIP_BASE + subpage_addr + i) != 0xFF) {
          is_written = true;
          break;
        }
      }

      // If we find data, record this as the last written subpage
      if (is_written) {
        last_written_page = page;
        last_written_subpage = subpage;
        break; // Stop scanning this page
      }
    }

    if (last_written_page != -1) {
      break; // Stop scanning once the last written page and subpage are found
    }
  }

  if (last_written_page == -1 || last_written_subpage == -1) {
    Serial.println("No configuration data found in flash memory.");
    return;
  }

  // Calculate the address of the last written subpage
  uint32_t last_written_addr = CONFIG_DATA + (last_written_page * FLASH_PAGE_SIZE) +
                               (last_written_subpage * SUBPAGE_SIZE);

  // Read and print the configuration data
  ConfigData config;
  memcpy(&config, (uint8_t *)(XIP_BASE + last_written_addr), sizeof(ConfigData));

  Serial.println("Last written configuration data:");
  Serial.println("Location: Page #" + String(last_written_page) +
                 ", Subpage #" + String(last_written_subpage) +
                 ", Address: 0x" + String(last_written_addr, DEC));
  printConfigData(&config);
}



// Helper function to print configuration data
void FlashStorage::printConfigData(const ConfigData *data) {
  Serial.println("Callsign: " + String(data->callsign, 6));
  Serial.println("Frequency: " + String(data->frequency, 6));
  Serial.println("Primary Delay: " + String(data->primary_delay));
  Serial.println("Secondary Delay: " + String(data->secondary_delay));
  Serial.println("Main Altitude: " + String(data->main_altitude));
  Serial.println("Backup State: " + String(data->backup_state));
  Serial.println("Pressure Transducer: " + String(data->pressure_transducer));
}




// Function to erase the flash memory in the allowed region
void FlashStorage::eraseFlashRegion(uint32_t start_addr, uint32_t end_addr) {
  Serial.println("Erasing flash region...");

  for (uint32_t sector_addr = start_addr; sector_addr < end_addr; sector_addr += FLASH_SECTOR_SIZE) {
    Serial.println("  Erasing sector at address: 0x" + String(sector_addr, DEC));
    flash_range_erase(sector_addr - XIP_BASE, FLASH_SECTOR_SIZE);
  }

  Serial.println("Flash region erased.");
}




int FlashStorage::findNextEmptyPageWithErase() {
  // Flash boundaries
  const uint32_t start_addr = SensorSector;
  const uint32_t end_addr = FileSystemEnd;
  const int pages_per_sector = 16; // Assuming 16 pages per sector
  const int total_sectors_in_range = (end_addr - start_addr) / (FLASH_PAGE_SIZE * pages_per_sector);

  Serial.println("\n=== Scanning for Next Empty Page ===");
  Serial.println("Start Address: 0x" + String(start_addr, HEX));
  Serial.println("End Address: 0x" + String(end_addr, HEX));
  Serial.println("Total Sectors in Range: " + String(total_sectors_in_range));

  int first_empty_page = -1;

  // Scan for empty pages
  for (uint32_t page_index = 0; page_index < (end_addr - start_addr) / FLASH_PAGE_SIZE; page_index++) {
    uint32_t page_addr = start_addr + (page_index * FLASH_PAGE_SIZE);

    // Ensure we are within bounds
    if (page_addr >= end_addr) {
      Serial.println("Address out of bounds: 0x" + String(page_addr, HEX));
      continue;
    }

    // Calculate sector and page within sector
    int sector_index = page_index / pages_per_sector;
    int page_within_sector = page_index % pages_per_sector;

    volatile uint8_t *p = (volatile uint8_t *)(XIP_BASE + page_addr);

    // Debug output: Print first 4 bytes of the page
    Serial.print("Checking Sector " + String(sector_index) + " Page " + String(page_within_sector) + " (Address: 0x");
    Serial.print(String(page_addr, HEX) + ") = ");
    for (int i = 0; i < 4; i++) {
      Serial.print(String(*(p + i), HEX) + " ");
    }
    Serial.println();

    // Check if the page is empty (all bytes 0xFF)
    bool is_empty = true;
    for (uint32_t i = 0; i < 4; i++) {
      if (*(p + i) != 0xFF) {
        is_empty = false;
        break;
      }
    }

    if (is_empty) {
      first_empty_page = page_index;
      g_firstEmptyPageAddr = page_addr;
      g_firstEmptyPageIndex = first_empty_page;

      // Store sector and page within sector
      g_firstEmptySectorIndex = sector_index;
      g_pageWithinSector = page_within_sector;

      // Compute the sector index relative to the range from SensorSector to end of flash
      g_sectorInRangeIndex = g_firstEmptySectorIndex;

      Serial.println("\n--- First Empty Page Found ---");
      Serial.println("Sector: " + String(g_firstEmptySectorIndex));
      Serial.println("Page Within Sector: " + String(g_pageWithinSector));
      Serial.println("Page Address: 0x" + String(g_firstEmptyPageAddr, HEX));
      Serial.println("Sector Index (Relative to Range): " + String(g_sectorInRangeIndex));
      Serial.println("-----------------------------\n");

      return g_firstEmptyPageAddr;
    }
  }

  // If no empty page is found, erase the flash region
  Serial.println("No empty page found. Erasing flash region and rechecking...");
  eraseFlashRegion(start_addr, end_addr);

  // Recheck after erasing
  for (uint32_t page_index = 0; page_index < (end_addr - start_addr) / FLASH_PAGE_SIZE; page_index++) {
    uint32_t page_addr = start_addr + (page_index * FLASH_PAGE_SIZE);

    // Ensure we are within bounds
    if (page_addr >= end_addr) {
      Serial.println("Address out of bounds: 0x" + String(page_addr, HEX));
      continue;
    }

    // Calculate sector and page within sector
    int sector_index = page_index / pages_per_sector;
    int page_within_sector = page_index % pages_per_sector;

    volatile uint8_t *p = (volatile uint8_t *)(XIP_BASE + page_addr);

    // Debug output: Print first 4 bytes of the page
    Serial.print("Rechecking Sector " + String(sector_index) + " Page " + String(page_within_sector) + " (Address: 0x");
    Serial.print(String(page_addr, HEX) + ") = ");
    for (int i = 0; i < 4; i++) {
      Serial.print(String(*(p + i), HEX) + " ");
    }
    Serial.println();

    // Check if the page is empty (all bytes 0xFF)
    bool is_empty = true;
    for (uint32_t i = 0; i < 4; i++) {
      if (*(p + i) != 0xFF) {
        is_empty = false;
        break;
      }
    }

    if (is_empty) {
      first_empty_page = page_index;
      g_firstEmptyPageAddr = page_addr;
      g_firstEmptyPageIndex = first_empty_page;

      // Store sector and page within sector
      g_firstEmptySectorIndex = sector_index;
      g_pageWithinSector = page_within_sector;

      // Compute the sector index relative to the range from SensorSector to end of flash
      g_sectorInRangeIndex = g_firstEmptySectorIndex;

      Serial.println("\n--- First Empty Page Found After Erase ---");
      Serial.println("Sector: " + String(g_firstEmptySectorIndex));
      Serial.println("Page Within Sector: " + String(g_pageWithinSector));
      Serial.println("Page Address: 0x" + String(g_firstEmptyPageAddr, HEX));
      Serial.println("Sector Index (Relative to Range): " + String(g_sectorInRangeIndex));
      Serial.println("-----------------------------------------\n");

      return g_firstEmptyPageAddr;
    }
  }

  Serial.println("No empty page found even after erasing flash region.");
  return -1;
}



void FlashStorage::writeToFlash(const uint8_t* data, size_t dataSize) {
    if (g_pageWithinSector < 0 || g_sectorInRangeIndex < 0) {
        Serial.println("Error: Invalid flash page/sector index.");
        return;
    }

    // Compute the actual address to write to
    uint32_t target_address = SensorSector + (g_sectorInRangeIndex * FLASH_SECTOR_SIZE) + (g_pageWithinSector * FLASH_PAGE_SIZE);

    // Check if we have exceeded FileSystemEnd and roll over if necessary
    if (target_address >= FileSystemEnd) {
        Serial.println("Reached end of flash range, rolling over...");
        g_sectorInRangeIndex = 0;
        g_pageWithinSector = 0;
        target_address = SensorSector; // Reset to the start address
    }

    // Ensure we don't write more than a page at once
    if (dataSize > FLASH_PAGE_SIZE) {
        Serial.println("Error: Data size exceeds flash page size.");
        return;
    }

    // Prepare the buffer (clear first)
    uint8_t buf[FLASH_PAGE_SIZE] = {0}; 
    memcpy(buf, data, dataSize);  // Copy incoming data into buffer

    // Perform the write operation
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(target_address, buf, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    Serial.print("Data written to address: 0x");
    Serial.println(target_address, HEX);

    // Update the page within the sector
    g_pageWithinSector++;

    // If the sector is full, move to the next sector
    if (g_pageWithinSector >= (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)) {
        g_pageWithinSector = 0;
        g_sectorInRangeIndex++;

        // If we exceed the file system range, roll over
        if ((SensorSector + (g_sectorInRangeIndex * FLASH_SECTOR_SIZE)) >= FileSystemEnd) {
            Serial.println("Rolling over to the first sector...");
            g_sectorInRangeIndex = 0;
        }

        // Check the next two sectors before moving
        int nextSector1 = g_sectorInRangeIndex;
        int nextSector2 = (g_sectorInRangeIndex + 1) % (FileSystemEnd / FLASH_SECTOR_SIZE);

        Serial.print("Checking sectors ");
        Serial.print(nextSector1);
        Serial.print(" and ");
        Serial.println(nextSector2);

        if (!checkSector(nextSector1)) {
            Serial.print("Erasing sector ");
            Serial.println(nextSector1);
            eraseSector(nextSector1);
        }

        if (!checkSector(nextSector2)) {
            Serial.print("Erasing sector ");
            Serial.println(nextSector2);
            eraseSector(nextSector2);
        }
    }
}




void FlashStorage::eraseSector(int sectorIndex) {
    // Calculate the sector's starting address
    uint32_t sector_address = SensorSector + (sectorIndex * FLASH_SECTOR_SIZE);

    // Ensure the sector is within the valid range
    if (sector_address < SensorSector || sector_address >= FileSystemEnd) {
        Serial.println("Error: Sector index out of range.");
        return;
    }

    Serial.print("Erasing sector #");
    Serial.print(sectorIndex);
    Serial.print(" at address 0x");
    Serial.println(sector_address, HEX);

    // Disable interrupts, erase the sector, then restore interrupts
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(sector_address, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    Serial.println("Sector erased.");
}



void FlashStorage::readPage(int sectorIndex, int pageIndex) {
    // Calculate the page's flash memory address
    uint32_t page_address = SensorSector + (sectorIndex * FLASH_SECTOR_SIZE) + (pageIndex * FLASH_PAGE_SIZE);

    // Ensure the address is within a valid range
    if (page_address < SensorSector || page_address >= FileSystemEnd) {
        Serial.println("Error: Page index out of range.");
        return;
    }

    // Adjust for XIP (memory-mapped flash)
    uint32_t address = XIP_BASE + page_address;

    // Print sector and page info
    Serial.print("Reading page ");
    Serial.print(pageIndex);
    Serial.print(" in sector ");
    Serial.print(sectorIndex);
    Serial.print(" at address 0x");
    Serial.println(address, HEX);

    // Ensure proper memory alignment
    if (address % sizeof(int) != 0) {
        Serial.println("Error: Misaligned memory access!");
        return;
    }

    // Read memory
    volatile uint32_t *p = (volatile uint32_t *)address;

    // Print the first four bytes in hex
    Serial.print("First four bytes: 0x");
    Serial.println(*p, HEX);
}



bool FlashStorage::checkSector(int sectorIndex) {
    bool isEmpty = true; // Assume the sector is empty unless proven otherwise

    Serial.print("Checking sector ");
    Serial.print(sectorIndex);
    Serial.println(":");

    // Iterate through all pages in the sector
    for (int pageIndex = 0; pageIndex < SECTOR_PAGE_COUNT; pageIndex++) {
        // Calculate the page's flash memory address
        uint32_t page_address = SensorSector + (sectorIndex * FLASH_SECTOR_SIZE) + (pageIndex * FLASH_PAGE_SIZE);

        // Ensure the address is within a valid range
        if (page_address < SensorSector || page_address >= FileSystemEnd) {
            Serial.println("Error: Page index out of range.");
            return false;
        }

        // Adjust for XIP (memory-mapped flash)
        uint32_t address = XIP_BASE + page_address;

        // Ensure proper memory alignment
        if (address % sizeof(int) != 0) {
            Serial.println("Error: Misaligned memory access!");
            return false;
        }

        // Read memory
        volatile uint32_t *p = (volatile uint32_t *)address;

        // Print the first four bytes for debugging
        Serial.print("Page ");
        Serial.print(pageIndex);
        Serial.print(" (0x");
        Serial.print(address, HEX);
        Serial.print("): 0x");
        Serial.println(*p, HEX);

        // If any page contains data other than 0xFFFFFFFF, mark sector as not empty
        if (*p != 0xFFFFFFFF) {
            isEmpty = false;
        }
    }

    return isEmpty;
}



// Adds data to the buffer, writes when full
void FlashStorage::addToBuffer(const void* data, size_t dataSize) {
    const uint8_t* byteData = (const uint8_t*)data;
    size_t bytesWritten = 0;

    // On the first run, flip the next bit
    if (firstWrite) {
        flipNextBit();
    }

    while (bytesWritten < dataSize) {
        size_t spaceLeft = FLASH_BUFFER_SIZE - bufferIndex;
        size_t bytesToCopy = min(spaceLeft, dataSize - bytesWritten);

        // If it's the first write of the session, add the 4-byte marker and FlightNumber
        if (firstWrite && bufferIndex == 0) {
            memcpy(flashBuffer, &START_MARKER, sizeof(START_MARKER));
            bufferIndex += sizeof(START_MARKER);

            memcpy(&flashBuffer[bufferIndex], &FlightNumber, sizeof(FlightNumber));
            bufferIndex += sizeof(FlightNumber);

            firstWrite = false;  // Reset flag so it doesn't write again
        }

        // Copy data to buffer
        memcpy(&flashBuffer[bufferIndex], &byteData[bytesWritten], bytesToCopy);
        bufferIndex += bytesToCopy;
        bytesWritten += bytesToCopy;

        // If buffer is full, write to flash and reset
        if (bufferIndex >= FLASH_BUFFER_SIZE) {
            writeToFlash(flashBuffer, FLASH_BUFFER_SIZE);
            bufferIndex = 0;  // Reset buffer index
        }
    }
}


void FlashStorage::scanFlashForRegions() {
  const uint32_t startMarker = START_MARKER;
  const uint32_t startAddr = SensorSector;
  const uint32_t endAddr = FileSystemEnd;
  const int pagesPerSector = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE;
  const int totalPages = (endAddr - startAddr) / FLASH_PAGE_SIZE;

  int firstEmptyPageAddr = findNextEmptyPageWithErase();
  if (firstEmptyPageAddr == -1) {
      Serial.println("Error: No empty pages found.");
      return;
  }

  int firstEmptyPage = (firstEmptyPageAddr - startAddr) / FLASH_PAGE_SIZE;
  int firstEmptySector = firstEmptyPage / pagesPerSector;

  regionCount = 0;
  int scanPage = firstEmptyPage;
  int scanSector = firstEmptySector;
  bool inRegion = false;
  SensorDataRegion currentRegion;

  int pagesScanned = 0;

  while (pagesScanned < totalPages) {
      uint32_t pageAddr = startAddr + (scanPage * FLASH_PAGE_SIZE);
      volatile uint32_t *p = (volatile uint32_t *)(XIP_BASE + pageAddr);

      if (*p == startMarker) {
          uint32_t regionID = *(p + 1);
          if (inRegion) {
              currentRegion.endPage = (scanPage % pagesPerSector);
              currentRegion.endSector = scanSector;
              if (regionCount < MAX_REGIONS) {
                  regions[regionCount++] = currentRegion;
              }
              Serial.println();
          }

          currentRegion.startPage = (scanPage % pagesPerSector);
          currentRegion.startSector = scanSector;
          currentRegion.regionID = regionID;
          inRegion = true;
      }

      if (inRegion) {
          for (int j = 0; j < FLASH_PAGE_SIZE; j++) {
              uint8_t byteData = *((volatile uint8_t *)(XIP_BASE + pageAddr + j));
              Serial.print(byteData, HEX);
              Serial.print(" ");
          }
      }

      bool isEmpty = true;
      for (int j = 0; j < FLASH_PAGE_SIZE; j++) {
          if (*((volatile uint8_t *)(XIP_BASE + pageAddr + j)) != 0xFF) {
              isEmpty = false;
              break;
          }
      }

      if (isEmpty && inRegion) {
          currentRegion.endPage = (scanPage % pagesPerSector);
          currentRegion.endSector = scanSector;
          if (regionCount < MAX_REGIONS) {
              regions[regionCount++] = currentRegion;
          }
          inRegion = false;
          Serial.println();
      }

      scanPage++;
      if (scanPage >= totalPages) {
          scanPage = 0;
          scanSector = 0;
      } else if (scanPage % pagesPerSector == 0) {
          scanSector++;
      }

      pagesScanned++;
  }

  if (inRegion && regionCount < MAX_REGIONS) {
      currentRegion.endPage = (scanPage % pagesPerSector);
      currentRegion.endSector = scanSector;
      regions[regionCount++] = currentRegion;
      Serial.println();
  }
}






void FlashStorage::viewSensorDataRegion(int regionIndex) {
    if (regionIndex < 0 || regionIndex >= regionCount) {
        Serial.println("Invalid region index.");
        return;
    }

    SensorDataRegion region = regions[regionIndex];
    Serial.println("\n=== Viewing Sensor Data Region ===");
    Serial.print("Region ");
    Serial.print(regionIndex + 1);
    Serial.print(": Start Sector ");
    Serial.print(region.startSector);
    Serial.print(", Page ");
    Serial.println(region.startPage);

    for (int sector = region.startSector; sector <= region.endSector; sector++) {
        int startPage = (sector == region.startSector) ? region.startPage : 0;
        int endPage = (sector == region.endSector) ? region.endPage - 1 : (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE - 1);

        for (int page = startPage; page <= endPage; page++) {
            uint32_t address = SensorSector + (sector * FLASH_SECTOR_SIZE) + (page * FLASH_PAGE_SIZE);
            uint8_t *p = (uint8_t *)(XIP_BASE + address);

            Serial.print("Sector ");
            Serial.print(sector);
            Serial.print(" Page ");
            Serial.print(page);
            Serial.print(" (Address: 0x");
            Serial.print(address, HEX);
            Serial.println(")");

            // Print full 256 bytes
            for (int i = 0; i < FLASH_PAGE_SIZE; i++) {
                Serial.print(p[i], HEX);
                Serial.print(" ");
                if ((i + 1) % 16 == 0) Serial.println();  // Print 16 bytes per line
            }
            Serial.println("\n");
        }
    }
}






void FlashStorage::requestUserRegionSelection() {
    if (regionCount == 0) {
        Serial.println("No sensor data regions found.");
        return;
    }

    Serial.println("\nEnter the region number (1-" + String(regionCount) + ") to view:");
    while (Serial.available() == 0); // Wait for user input

    int selectedRegion = Serial.parseInt() - 1;
    viewSensorDataRegion(selectedRegion);
}




void FlashStorage::flipNextBit() {
    Serial.println("Flipping the next bit in FlightSector...");

    const uint32_t total_bits = FLASH_SECTOR_SIZE * 8; // 32768 bits
    const uint32_t total_pages = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; // 16 pages
    uint8_t page_buffer[FLASH_PAGE_SIZE]; // 256-byte page buffer

    uint32_t bit_position = 0;
    uint32_t byte_index = 0;
    uint8_t bit_index = 0;
    uint32_t page_address = 0;
    bool sector_full = true;

    // Scan for the first available bit to flip
    for (uint32_t page = 0; page < total_pages; page++) {
        page_address = FlightSector + (page * FLASH_PAGE_SIZE);
        memcpy(page_buffer, (uint8_t *)(XIP_BASE + page_address), FLASH_PAGE_SIZE);

        for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++) {
            if (page_buffer[i] != 0x00) { // If any bit is still 1
                sector_full = false;

                for (bit_index = 0; bit_index < 8; bit_index++) {
                    if (page_buffer[i] & (1 << (7 - bit_index))) { // Find first '1' bit
                        byte_index = i;
                        bit_position = (page * FLASH_PAGE_SIZE * 8) + (i * 8) + bit_index;
                        goto FOUND_BIT;
                    }
                }
            }
        }
    }

    // If full, erase and restart
    if (sector_full) {
        Serial.println("Sector full, erasing...");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FlightSector, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        Serial.println("Sector erased, restarting bit flipping from bit 0.");
        return;
    }

FOUND_BIT:
    // Flip the found bit
    page_buffer[byte_index] &= ~(1 << (7 - bit_index));

    // Write full 256-byte page back
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(page_address, page_buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    Serial.print("Flipped bit at absolute position: ");
    Serial.print(bit_position);
    Serial.print(" (Page: ");
    Serial.print(page_address / FLASH_PAGE_SIZE);
    Serial.print(", Byte index: ");
    Serial.print(byte_index);
    Serial.print(", Bit index: ");
    Serial.print(bit_index);
    //set global variable for flight number
    FlightNumber = bit_position + 1;
    Serial.println(")");
}




/*
bool FlashStorage::readCurrentBit() {
    Serial.println("Reading the most recently flipped bit...");

    const uint32_t total_bits = FLASH_SECTOR_SIZE * 8; // 32768 bits
    const uint32_t total_pages = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; // 16 pages
    uint8_t page_buffer[FLASH_PAGE_SIZE];

    for (int page = total_pages - 1; page >= 0; page--) {
        uint32_t page_address = FlightSector + (page * FLASH_PAGE_SIZE);
        memcpy(page_buffer, (uint8_t *)(XIP_BASE + page_address), FLASH_PAGE_SIZE);

        for (int i = FLASH_PAGE_SIZE - 1; i >= 0; i--) {
            for (int bit_index = 7; bit_index >= 0; bit_index--) {
                if ((page_buffer[i] & (1 << (7 - bit_index))) == 0) { // Found flipped bit
                    uint32_t bit_position = (page * FLASH_PAGE_SIZE * 8) + (i * 8) + bit_index;
                    Serial.print("Most recently flipped bit at absolute position: ");
                    Serial.print(bit_position);
                    Serial.print(" (Page: ");
                    Serial.print(page);
                    Serial.print(", Byte index: ");
                    Serial.print(i);
                    Serial.print(", Bit index: ");
                    Serial.print(bit_index);
                    Serial.println(")");
                    //set global variable for flight number
                    FlightNumber = bit_index + 1;
                    return false; // Found flipped bit
                }
            }
        }
    }

    Serial.println("No flipped bits found, sector may be empty.");
    return true; // Sector is empty
}
*/