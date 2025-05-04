#include "BLEDevice.h"
#include <esp_task_wdt.h>
#include <cmath>
#include <string>
#include <map>
#include <Time.h>
#include <TimeLib.h>
#include "WiFi.h"
#include <WiFiClient.h>
#include "AdafruitIO_WiFi.h"
#define FLORA_ADDR_N1 "c4:7c:8d:6a:e2:30"
#define FLORA_ADDR_N2 "c4:7c:8d:6a:5c:b8"
#define WIFI_SSID "pacopaco"
#define WIFI_PASS "taratata12"
#define IO_USERNAME "vciriza"
#define IO_KEY "bc5dd8be8da94dbe94fa0dabad22804d"
#if defined(USE_AIRLIFT) || defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL)
// Configure the pins used for the ESP32 connection
#if !defined(SPIWIFI_SS)  // if the wifi definition isnt in the board variant
// Don't change the names of these #define's! they match the variant ones
#define SPIWIFI SPI
#define SPIWIFI_SS 10  // Chip select pin
#define NINA_ACK 9     // a.k.a BUSY or READY pin
#define NINA_RESETN 6  // Reset pin
#define NINA_GPIO0 -1  // Not connected
#endif
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS, SPIWIFI_SS,
                   NINA_ACK, NINA_RESETN, NINA_GPIO0, &SPIWIFI);
#else
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
#endif

#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 300       //Time ESP32 will go to sleep (in seconds)

int freeHeapstart;
int freeHeapstop;
int c;

// Define the task handle
TaskHandle_t taskHandle;
const unsigned long interval = 180000;  // Interval to wait (3 minutes)

// Usage:
// Select your ESP32 board under: Tools > "Board: [current selected board]" > i.e. "NodeMCU-32s"
// Connect your ESP32 via USB
// Open Serial Monitor: Tools > Serial Monitor
// On bottom set Serial Monitor to: 115200 baud for receiving output
// Then upload to your ESP 32

// Resource
// For reading and writing values, as well as data structure: https://github.com/vrachieru/xiaomi-flower-care-api
// For BLE connection:

// Uncomment to get more information printed out
 #define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTF(x,y)  Serial.printf(x,y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTF(x,y)
#endif

// Service #0: Root service used for advertising of Bluetooth low energy (BLE) connection
static BLEUUID serviceAdvertisementUUID("0000fe95-0000-1000-8000-00805f9b34fb");

// Service #1: Real-time sensor data, battery level and firmware version
// Real-time sensor data with properties: READ, WRITE, NOTIFY 	
static BLEUUID serviceBatteryFirmwareRealTimeDataUUID("00001204-0000-1000-8000-00805f9b34fb");
// Read request needs to be written prior to be able to read real-time data.
uint8_t CMD_REAL_TIME_READ_INIT[2] = {0xA0, 0x1F};                                                  //  write these two bytes...
static BLEUUID characteristicReadRequestToRealTimeDataUUID("00001a00-0000-1000-8000-00805f9b34fb"); // ...to this characteristic 
static BLEUUID characteristicRealTimeDataUUID("00001a01-0000-1000-8000-00805f9b34fb");              // ...in order to read actual values of this characteristic
// Firmware version and battery level with properties: READ can be directly read returning battery level in % and firmware version in '3.2.2' notation.
static BLEUUID characteristicFirmwareAndBatteryUUID("00001a02-0000-1000-8000-00805f9b34fb");

// Service #2: Epoch time and Historical data
static BLEUUID serviceHistoricalDataUUID("00001206-0000-1000-8000-00805f9b34fb");
// Read request needs to be written prior to be able to read real-time data.
uint8_t CMD_HISTORY_READ_INIT[3] = {0xa0, 0x00, 0x00};                                              // write these three bytes...
static BLEUUID characteristicReadRequestToHistoricalDataUUID("00001a10-0000-1000-8000-00805f9b34fb");// ...to this characteristic (Write Characteristic)
static BLEUUID characteristicHistoricalDataUUID("00001a11-0000-1000-8000-00805f9b34fb");            // ...in order to read number of historical entries (Read Characteristic)
// afterwards you can write the history entry addresses to Write Characteristic and then read the value stored in the Read Characteristic one by one
uint8_t CMD_HISTORY_READ_ENTRY[3] = {0xa1, 0x00, 0x00}; // 0xa1 is fixed, 0x00 00 represents entry #1, 0x01 00 represents entry #2, 0x02 00 entry #3, ...
// Epoch time values can be directly read of this UUID returning time in seconds since device boot.
static BLEUUID characteristicEpochTimeUUID("00001a12-0000-1000-8000-00805f9b34fb");

// Then connect to server
static BLEAdvertisedDevice* myDevice;
static BLEClient*  pClient;

// Flags
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

// CLASSES -------------------------------------------------------------------------------------------

// A real-time entry ouput example of 16 bytes in Hex: "0E 01 00 48 02 00 00 28 D0 00 02 3C 00 FB 34 9B". 
// If using a hex converter, remember to swap bytes order (little endian encoded) to gain correct values.
class RealTimeEntry
{
  public:
  RealTimeEntry(std::string value)
  {
    const char *val = value.c_str();
    raw_val = value;
    temperature = ((int16_t*) val)[0];    // 2 bytes at pos 00-01: "0E 01"        -> 270 * 0.1°C = 27.0 °C
    // 1 byte at pos 02 unknown, seems to be fixed value of "0x00"
    brightness = *(uint32_t*) (val+3);    // 4 bytes at pos 03-06: "48 02 00 00"  -> 584 lux
    moisture = *(uint8_t*) (val+7);       // 1 byte  at pos 07:    "28"           -> 40%
    conductivity = *(uint16_t*) (val+8);  // 2 bytes at pos 08-09: "D0 00"        -> 208 µS/cm
    // 6 bytes at pos 10-15 unknown, seems to be fixed value of "0x02 3C 00 FB 34 9B" 
  }
  std::string toString()
  {
    char buffer[120];
    int count = 0;
    count += snprintf(buffer, sizeof(buffer), "Temperature: %2.1f °C\n", ((float)this->temperature)/10);
    count += snprintf(buffer+count, sizeof(buffer)-count, "Brightness: %u lux\n", this->brightness);
    count += snprintf(buffer+count, sizeof(buffer)-count, "Soil moisture: %d %s\n", this->moisture, "%");
    count += snprintf(buffer+count, sizeof(buffer)-count, "Conductivity: %" PRIu16 " µS/cm \n", this->conductivity);
    return (std::string)buffer;
  }
  std::string raw_val;
  int16_t temperature; // in 0.1 °C
  uint32_t brightness; // in lux
  uint8_t moisture; // in %
  uint16_t conductivity; // in µS/cm
};


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // We have found a device, let us now see if it contains the service we are looking for.
    DEBUG_PRINT("Found a BLE device, checking if service UUID is present...");
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceAdvertisementUUID)) {
      Serial.printf("BLE Advertised Device found: \n%s\n", advertisedDevice.toString().c_str());

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
    else
      DEBUG_PRINT("Service not found on this BLE device.\n");
  } // onResult
}; // MyAdvertisedDeviceCallbacks

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    Serial.printf("Callback: onConnect.\n\n");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.printf("Callback: onDisconnect.\n\n");
  }
};

class FlowerCare {
  public:
  FlowerCare()
  {
    scanBLE();
    if(myDevice->haveName())
      _name = std::string(myDevice->getName().c_str());
    else
      _name = "Unnamed";

    _macAddress = std::string(myDevice->getAddress().toString().c_str());
  }

  void scanBLE()
  {
    Serial.printf("Scanning for BLE devices with serviceAdvertisementUUID =  %s...\n", serviceAdvertisementUUID.toString().c_str());
    BLEDevice::init("");
    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device.  Specify that we want active scanning and start the
    // scan to run for 5 seconds.
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
    delay(1000);
    Serial.printf("Scanning done.\n\n");
  }

  void connectToServer()
  {
      Serial.printf("Create client and connecting to %s\n", myDevice->getAddress().toString().c_str());
      pClient  = BLEDevice::createClient();
      pClient->setClientCallbacks(new MyClientCallback());
      pClient->connect(myDevice);  // Connect to the remove BLE Server.
  }

  // Device's mac address usually starts with: "c4:7c:8d:xx:xx:xx"
  std::string getMacAddress()
  {
    return _macAddress;
  }

  // The name usually is "Flower care"
  std::string getName()
  {
    return _name;
  }
  
  // Firmware and battery ouput example in Hex: "64 2B 33 2E 32 2E 32". 
  // Battery level is stored in very first hex: "64" represented in int as: 100%.
  int getBatteryLevel()
  {
     std::string value = readValue(serviceBatteryFirmwareRealTimeDataUUID, characteristicFirmwareAndBatteryUUID);
     _batteryLevel = (uint8_t)value[0];
     //battery = _batteryLevel; 
     return _batteryLevel;
  }
  
  // Firmware version stored in positions 2 till end: "33 2E 32 2E 32" represented in ASCII Text: 3.2.2
  std::string getFirmwareVersion()
  {
    if(_firmwareVersion.empty())
    {
      std::string value = readValue(serviceBatteryFirmwareRealTimeDataUUID, characteristicFirmwareAndBatteryUUID);
      _firmwareVersion = &value[2]; 
      return _firmwareVersion;
    }
  }

  RealTimeEntry* getRealTimeData()
  {
    writeValue(serviceBatteryFirmwareRealTimeDataUUID, characteristicReadRequestToRealTimeDataUUID, CMD_REAL_TIME_READ_INIT, 2);
    delay(2000);
    std::string value = readValue(serviceBatteryFirmwareRealTimeDataUUID, characteristicRealTimeDataUUID);
    return new RealTimeEntry(value);
  }


  void printSecondsInDays(int n)
  {
    int days = n / (24*3600);
    n = n % (24*3600);
    int hours = n / 3600;
    n %= 3600;
    int minutes = n / 60;
    n %= 60;
    int seconds = n / 60;
    Serial.printf("%d days, %d hours, %d minutes, %d seconds.\n", days, hours, minutes, seconds);
  }
  
  private:
  std::string _macAddress = "c4:7c:xx:xx:xx:xx";
  std::string _name;
  uint8_t _batteryLevel;
  std::string _firmwareVersion = "";
  uint32_t _epochTime;

  std::string readValue(BLEUUID serviceUUID, BLEUUID characteristicUUID)
  {
    printDebugReadValue(serviceUUID.toString().c_str(), characteristicUUID.toString().c_str());
    std::string value = "";
    BLERemoteService* pService = getService(serviceUUID);
    if (pService == nullptr) {
      Serial.println("Returning empty string value.");
      return value;
    }
    BLERemoteCharacteristic* pCharacteristic = getCharacteristic(pService, characteristicUUID);
    if(pCharacteristic == nullptr) {
      Serial.println("Returning empty string value.");
      return value;
    }
    try
    {
      value = std::string(pCharacteristic->readValue().c_str());
      printDebugHexValue(value, value.length());
    }
    catch(...)
    {
      Serial.println("ERROR: Failed reading value of characteristic.");
    }
    return value;
  }

  BLERemoteService* getService(BLEUUID serviceUUID)
  {
    BLERemoteService* pService = pClient->getService(serviceUUID);
    if (pService == nullptr)
        Serial.printf("ERROR: Failed to get service UUID: %s. Check if UUID is correct.\n", serviceUUID.toString().c_str());
    return pService;
  }

  BLERemoteCharacteristic* getCharacteristic(BLERemoteService* pService, BLEUUID characteristicUUID)
  {
    BLERemoteCharacteristic* pCharacteristic = pService->getCharacteristic(characteristicUUID);
    if(pCharacteristic == nullptr)
      Serial.printf("ERROR: Failed to get characteristic UUID: %s. Check if the characteristic UUID is correct and exists in given service UUID: %s.\n", characteristicUUID.toString().c_str(), pService->getUUID().toString().c_str());
    return pCharacteristic;
  }

  void writeValue(BLEUUID serviceUUID, BLEUUID characteristicUUID, uint8_t* pCMD, int cmdLength)
  {
    printDebugWriteValue(std::string(serviceUUID.toString().c_str()),
                     std::string(characteristicUUID.toString().c_str()),
                     pCMD, cmdLength);
    BLERemoteService* pService = getService(serviceUUID);
    BLERemoteCharacteristic* pCharacteristic = pService->getCharacteristic(characteristicUUID);
    pCharacteristic->writeValue(pCMD, cmdLength, true);
  }
  
  void disconnectOnError()
  {
    Serial.println("Disconnecting BLE connection due to error.");
    pClient->disconnect();
    connected = false;
  }
  

  // Debug prints--------------------------------------------------------------------------------------------
  void printDebugWriteValue(std::string serviceUUID, std::string characteristicUUID, uint8_t cmd[], int cmdLength)
  {
    #ifdef DEBUG
    Serial.printf("DEBUG: Writing the following %d bytes: ' ", cmdLength);
    for(int i = 0; i < cmdLength; i++)
      Serial.printf("%02x ", cmd[i] & 0xff);
    Serial.printf("' to characteristicUUID = %s of serviceUUID = %s \n", characteristicUUID.c_str(), serviceUUID.c_str());
    #endif
  }

  void printDebugReadValue(std::string serviceUUID, std::string characteristicUUID)
  {
    #ifdef DEBUG
    Serial.printf("DEBUG: Reading value of characteristicUUID = %s of serviceUUID = %s:\n", serviceUUID.c_str(), characteristicUUID.c_str()); 
    #endif
  }

  void printDebugHexValue(std::string value, int len)
  {
    #ifdef DEBUG
     Serial.printf("DEBUG: Value length n = %d, Hex: ", len);
      for (int i = 0; i < len; i++)
        Serial.printf("%02x ", (int)value[i]); 
    Serial.println(" ");
    #endif
  }
};

static FlowerCare* flowerCare;

std::string printDeviceLabel(const std::string& mac_address) {
  // Map to associate MAC addresses with labels
  static const std::map<std::string, std::string> mac_to_label = {
    {FLORA_ADDR_N1, "N1"},
    {FLORA_ADDR_N2, "N2"}
  };

  // Try to find the MAC address in the map
  auto it = mac_to_label.find(mac_address);
  if (it != mac_to_label.end()) {
    return it->second; // Return the label if found
  } else {
    return "UNKNOWN"; // Return "UNKNOWN" if not found
  }
}

std::string getGroupNameForMac(const std::string& mac_address) {
  std::string n_is = printDeviceLabel(mac_address);
  return n_is + "_Flora_Sensor";
}

void getSensorKeysForMac(const std::string& mac_address,
                         std::string& temp_key,
                         std::string& light_key,
                         std::string& moisture_key,
                         std::string& fertility_key,
                         std::string& battery_key) {

  std::string prefix = printDeviceLabel(mac_address);                      
  temp_key = prefix + "_Flower_Temp_Sensor";
  light_key = prefix + "_Flower_Light_Sensor";
  moisture_key = prefix + "_Flower_Moisture_Sensor";
  fertility_key = prefix + "_Flower_Fertility_Sensor";
  battery_key = prefix + "_Flower_Battery_Sensor";
}


void sendSensorDataN(RealTimeEntry* rt_data, uint8_t bt_data, const std::string& mac_address) {
  if (rt_data == nullptr) {
    Serial.println("Error: rt_data is null");
    return;
  }

  printDeviceLabel(mac_address);

  std::string group_name = getGroupNameForMac(mac_address);
  Serial.print("Group name: ");
  Serial.println(group_name.c_str());

  Serial.print("Sending to AdafruitIO group: ");
  Serial.println(group_name.c_str());

  AdafruitIO_Group* group = io.group(group_name.c_str());

  io.connect();

  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  io.run();

  // Get sensor keys
  std::string temp_key, light_key, moisture_key, fertility_key, battery_key;
  getSensorKeysForMac(mac_address, temp_key, light_key, moisture_key, fertility_key, battery_key);

  // Convert temperature
  float tmp_f = ((float)rt_data->temperature) / 10.0;
  int16_t tmp_int = static_cast<int16_t>(round(tmp_f));

  // Send data
  group->set(temp_key.c_str(), tmp_int);
  group->set(light_key.c_str(), rt_data->brightness);
  group->set(moisture_key.c_str(), rt_data->moisture);
  group->set(fertility_key.c_str(), rt_data->conductivity);
  group->set(battery_key.c_str(), bt_data);
  group->save();

  Serial.println(io.statusText());
}

void printHex(const std::string &data) {
    for (unsigned char c : data) {
        Serial.printf("%02X ", c);
    }
    Serial.println();
}

void getSensorDataN() {
  Serial.println("Sending N2 to Adafruit");

  // Connect to flower care server
  if (doConnect == true) {
    flowerCare->connectToServer();
    doConnect = false;
  }

  if (connected == true) {
    Serial.printf("Name: %s\n", flowerCare->getName().c_str());
    std::string macadress = flowerCare->getMacAddress();
    Serial.printf("Mac address: %s\n", macadress.c_str());
    uint8_t btry_data = flowerCare->getBatteryLevel();
    Serial.printf("Battery level: %d %%\n", btry_data);
    Serial.printf("Firmware version: %s\n", flowerCare->getFirmwareVersion().c_str());

    RealTimeEntry* rt_data = flowerCare->getRealTimeData();
    Serial.println("Raw data:");
    printHex(rt_data->raw_val);
    Serial.printf("Real time data: \n%s\n", rt_data->toString().c_str());

    sendSensorDataN(rt_data, btry_data, macadress);
    delete rt_data;
    delay(500);
    connected = false;
  }

  Serial.println("ALL SENT to Adafruit");

  freeHeapstop = ESP.getFreeHeap();
  Serial.print("FREEHEAP END : ");
  Serial.println(freeHeapstop);

  deepsleep();  // Go to deep sleep after sending data
}

void task1(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.println("Before watchdog reset...");
  esp_task_wdt_reset();  // Reset the Task Watchdog Timer
  Serial.println("After watchdog reset...");
  WiFi.mode(WIFI_OFF);
  flowerCare = new FlowerCare();
  delay(500);  // delay 0.5s
  getSensorDataN();  // Collect sensor data
  deepsleep();       // Go to deep sleep after collecting data
}

void deepsleep() {
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(500);  // delay 0.5s
  // Set all LED pins LOW
  int ledPins[] = { 2, 38, 48 };  
  for (int i = 0; i < sizeof(ledPins) / sizeof(ledPins[0]); i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  Serial.println("dodo...");
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);

  // Initialize the watchdog timer
  esp_task_wdt_config_t config = {
    .timeout_ms = (uint32_t)(TIME_TO_SLEEP * uS_TO_S_FACTOR * 1.5), // Set timeout to x1.5 times deepsleep
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,     // Monitor all cores
    .trigger_panic = true                                // Trigger panic if watchdog timeout occurs
  };

  esp_task_wdt_reconfigure(&config);
}

void loop() {
  Serial.println("Starting Flora client...");
  while (1) {
    xTaskCreate(
      task1,       // Task function pointer
      "Task1",     // Task name
      4096,        // Stack depth in words
      NULL,        // Task parameter
      2,           // Task priority
      &taskHandle  // Task handle
    );
    delay(interval);  // Wait for the next cycle
  }
}


