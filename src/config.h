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
#define FW_VERSION "V1.202"
#define OTA_DATE "260614"

#define WORK_PACKAGE "1191"
#define DEVICE_TYPE "03"
#define DEVICE_CODE_UPLOAD_DATE "250616"
#define DEVICE_SERIAL_ID "0119"

#define UNIQUE_DEVICE_ID WORK_PACKAGE DEVICE_TYPE DEVICE_CODE_UPLOAD_DATE DEVICE_SERIAL_ID

bool deviceArmed = true;  // Default to armed, can be changed via MQTT command

//Duplicalte from gsm.h for global access
QueueHandle_t mqttPublishQueue = NULL;

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
const unsigned long HEARTBEAT_INTERVAL = 5 * 60 * 1000UL;
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
const char* MQTT_EM_HB = "DMA/EM/HB";
const char* MQTT_EM_PUB = "DMA/EM/PUB";
const char* MQTT_AC_HB = "DMA/MeshAC/HB";
const char* MQTT_AC_GW_HB = "DMA/MeshAC/GW/HB";
const char* MQTT_AC_SUB = "DMA/MeshAC/SUB/";
const char* MQTT_AC_ACK = "DMA/MeshAC/ACK";
const char* MQTT_AC_TMP = "DMA/MeshAC/TEMP";

struct MQTTMessage {
    char topic[50];
    char payload[128];
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
