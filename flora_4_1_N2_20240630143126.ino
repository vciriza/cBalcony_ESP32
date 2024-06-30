#include <esp_task_wdt.h>

#define WDT_TIMEOUT 60
#include "BLEDevice.h"
#define FLORA_ADDR_N1 "c4:7c:8d:6a:e2:30"
#define FLORA_ADDR_N2 "c4:7c:8d:6a:5c:b8"
#include <Time.h>
#include <TimeLib.h>
#include "WiFi.h"
#include <WiFiClient.h>
#include "AdafruitIO_WiFi.h"
#define WIFI_SSID "pacopaco"
#define WIFI_PASS "taratata12"
#define IO_USERNAME "vciriza"
#define IO_KEY "bc5dd8be8da94dbe94fa0dabad22804d"
#if defined(USE_AIRLIFT) || defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) ||         \
    defined(ADAFRUIT_PYPORTAL)
// Configure the pins used for the ESP32 connection
#if !defined(SPIWIFI_SS) // if the wifi definition isnt in the board variant
// Don't change the names of these #define's! they match the variant ones
#define SPIWIFI SPI
#define SPIWIFI_SS 10 // Chip select pin
#define NINA_ACK 9    // a.k.a BUSY or READY pin
#define NINA_RESETN 6 // Reset pin
#define NINA_GPIO0 -1 // Not connected
#endif
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS, SPIWIFI_SS,
                   NINA_ACK, NINA_RESETN, NINA_GPIO0, &SPIWIFI);
#else
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
#endif

#define uS_TO_S_FACTOR 1000000     //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  600          //Time ESP32 will go to sleep (in seconds)

BLEClient* pClient;

//**********TOKEN (OK)**********//
char auth[] = "rdqdVUHoRxfVnV6isiJKXjI3RfGqUW56";

//**********WIFI (OK)**********//
char ssid[] = "pacopaco";
char pass[] = "taratata12";

// The remote service we wish to connect to.
static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");

// The characteristic of the remote service we are interested in.
static BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");
static BLEAddress floraAddressN1(FLORA_ADDR_N1);
static BLEAddress floraAddressN2(FLORA_ADDR_N2);
static BLERemoteCharacteristic* pRemoteCharacteristic;
float temp;
float moisture;
float light;
int conductivity;
int freeHeapstart;
int freeHeapstop;
int c;

// Define the task handle
TaskHandle_t taskHandle;

unsigned long lastTime = 0;       // Store the last time getSensorData1 was called
const unsigned long interval = 180000;  // Interval to wait (3 minutes)

void getSensorData(BLEAddress pAddress) {
    btStart();
    Serial.println("========== Forming a connection to Flora device ==========");
    Serial.println(pAddress.toString().c_str());
    Serial.println(" - Connection to Flora");
    if (!pClient->connect(pAddress)) {
        pClient->disconnect();
        delay(500); // delay 0.5s
        Serial.println(" - Cannot connect to Flora");
        ESP.restart();
    } else {
        Serial.println(" - Connected to Flora");
    }
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) { 
        Serial.print("Failed to find our service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
    } else {
        Serial.println(" - Found our service");
    }
 
    pRemoteCharacteristic = pRemoteService->getCharacteristic(uuid_write_mode);
    uint8_t buf[2] = {0xA0, 0x1F};
    pRemoteCharacteristic->writeValue(buf, 2, true);
  
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(uuid_sensor_data);
    Serial.println(pRemoteService->toString().c_str());
    if (pRemoteCharacteristic == nullptr) { 
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(uuid_sensor_data.toString().c_str());
    } else {
        Serial.println(" - Found our characteristic");
    }
    
    // Read the value of the characteristic.
    std::string value = pRemoteCharacteristic->readValue().c_str();
    Serial.print("The characteristic value was: ");
    const char *val = value.c_str();

    Serial.print("Hex: ");
    for (int i = 0; i < 16; i++) { 
        Serial.print((int)val[i], HEX);
        Serial.print(" ");
    }
  
    Serial.println(" ");
    temp     = (val[0] + val[1] * 256) / ((float)10.0);
    moisture = val[7];
    light    = val[3] + val[4] * 256;
    conductivity = val[8] + val[9] * 256;
  
    char buffer[64];
    temp     = float(temp);
    moisture = float(moisture);
    light    = float(light);
    conductivity = float(conductivity);
  
    Serial.print("Temperature: ");
    Serial.println(temp);
    Serial.print("Moisture: ");
    Serial.println(moisture);
    Serial.print("Light: ");
    Serial.println(light);
    Serial.print("Conductivity: ");
    Serial.println(conductivity);
  
    pClient->disconnect();
    delay(500);           // delay 0.5s
    btStop();
    delay(500);           // delay 0.5s
    //if (pAddress==floraAddressN1){
    //  Serial.println("Send N1");
    //  sendSensorDataN1();     // FUNCTION 5 IN 3
    //}
    if (pAddress==floraAddressN2){
      Serial.println("Send N2");
      sendSensorDataN2();     // FUNCTION 5 IN 3
    }
}

void sendSensorDataN1() {
    Serial.print("Going for AdafruitIO for N1: ");
    AdafruitIO_Group *group = io.group("N1_Flora_Sensor");
    io.connect();
    // wait for a connection
    while(io.status() < AIO_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    io.run();
    group->set("N1_Flower_Temp_Sensor", temp);
    group->set("N1_Flower_Light_Sensor", light);
    group->set("N1_Flower_Moisture_Sensor", moisture);
    group->set("N1_Flower_Fertility_Sensor", conductivity);
    group->save();
    Serial.println(io.statusText());
}

void sendSensorDataN2() {
    Serial.print("Going for AdafruitIO for N2: ");
    AdafruitIO_Group *group = io.group("N2_Flora_Sensor");
    io.connect();
    // wait for a connection
    while(io.status() < AIO_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    io.run();
    group->set("N2_Flower_Temp_Sensor", temp);
    group->set("N2_Flower_Light_Sensor", light);
    group->set("N2_Flower_Moisture_Sensor", moisture);
    group->set("N2_Flower_Fertility_Sensor", conductivity);
    group->save();
    Serial.println(io.statusText());
}

void getSensorData1() {
    // Serial.println("Sending N1 to Adafruit");
    // getSensorData(floraAddressN1); // Start getting sensor data
    Serial.println("Sending N2 to Adafruit");
    getSensorData(floraAddressN2);
    Serial.println("ALL SENT to Adafruit");
    freeHeapstop = ESP.getFreeHeap();
    Serial.print("FREEHEAP END : ");
    Serial.println(freeHeapstop);
    deepsleep();          // Go to deep sleep after sending data
}

void deepsleep() {
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    delay(200); // delay 0.2s
    Serial.println("dodo...");
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);

    // Initialize the watchdog timer
    esp_task_wdt_config_t config = {
        .timeout_ms = TIME_TO_SLEEP * uS_TO_S_FACTOR * 1.5,  // Set timeout to x1.5 times deepsleep
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Monitor all cores
        .trigger_panic = true  // Trigger panic if watchdog timeout occurs
    };

    esp_task_wdt_reconfigure(&config);
    
}

void task1(void *parameter) {
    esp_task_wdt_add(NULL);
    Serial.println("Before watchdog reset...");
    esp_task_wdt_reset();  // Reset the Task Watchdog Timer
    Serial.println("After watchdog reset...");
    BLEDevice::init("");
    pClient  = BLEDevice::createClient();
    delay(500);            // delay 0.5s
    WiFi.mode(WIFI_OFF);    
    getSensorData1();      // Collect sensor data
    deepsleep();           // Go to deep sleep after collecting data
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
      delay(interval);       // Wait for the next cycle
    }
}
