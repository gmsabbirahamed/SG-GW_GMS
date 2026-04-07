#pragma once

#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h>

// ==================== Configuration ====================
#define OTA 1  // Set to 1 for OTA mode, 0 for normal mode
#define HW_VERSION "3.0"
#define FW_VERSION "V1.502"
#define OTA_DATE "260101"

#define WORK_PACKAGE "1102"
#define DEVICE_TYPE "03"
#define DEVICE_CODE_UPLOAD_DATE "250616"
#define DEVICE_SERIAL_ID "0117"

#define UNIQUE_DEVICE_ID WORK_PACKAGE DEVICE_TYPE DEVICE_CODE_UPLOAD_DATE DEVICE_SERIAL_ID



String DEVICE_ID = "";
String MAC_FALLBACK_ID = "";

// ==================== Button Configuration ====================
#define BUTTON_PIN 35
#define BUTTON_DEBOUNCE_INTERVAL 50  // 50ms debounce
#define DOUBLE_PRESS_TIMEOUT_MS 500  // Changed name to avoid conflict
#define HOLD_CHECK_INTERVAL 100       // Check hold every 100ms

// Serial and SIM-A7670 pin config
#define SerialMon Serial
#define SerialAT Serial1
#define MODEM_TX 17
#define MODEM_RX 16
#define MODEM_PWR 15
#define ACLINE_PIN 34

#define ON HIGH
#define OFF LOW

// Timing Configuration
const unsigned long HEARTBEAT_INTERVAL = 60000UL;
const unsigned long GSM_ERROR_RETRY_DELAY = 10000UL;
unsigned long lastHeartbeat = 0;
unsigned long lastGsmErrorTime = 0;
unsigned long buttonHoldStart = 0;
bool buttonHolding = false;

// System State
volatile bool mqttConnected = false;
volatile bool gprsConnected = false;
volatile bool deviceOnline = false;
volatile bool otaInProgress = false;
volatile bool tasksSuspended = false;
bool modemWasInitialized = false;

// ==================== Global Variables ====================
const char* pubTopic = "DMA/SG/PUB";
const char* subTopic = "DMA/SG/";
const char* hbTopic = "DMA/SG/PUB";

struct MQTTMessage {
    char topic[50];
    char payload[100];
};

// Helper functions
String getMACDeviceID();

String getMACDeviceID() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}
