#pragma once
// This code is designed to run on an ESP32 device with a SIM7600 GSM module.
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Update.h>
#include "led.h"

// Configuration
const char apn[] = "internet";
const char user[] = "";
const char pass[] = "";

// MQTT Configuration
const char* broker = "broker2.dma-bd.com";
const char* mqttUser = "broker2";
const char* mqttPass = "Secret!@#$1234";

// OTA URLs
// const char* otaHost = "ota.gmsabbirahamed.com";
// const int otaPort = 80;
// const char* versionPath = "/esp32/version.txt";
// const char* firmwarePath = "/esp32/firmware/blink/firmware.bin";

String firmwareUrl = "";
// const char* defaultFirmwareUrl = "http://ota.gmsabbirahamed.com/esp32/firmware/blink/firmware.bin";
const char* defaultFirmwareUrl = "http://iot2.dma-bd.com:5000/download/iotsecurityv3101.bin";

//Task Handles
TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;

//Queue Handles
QueueHandle_t mqttPublishQueue = NULL;

// Mutexes Handles
SemaphoreHandle_t mqttMutex = NULL;
SemaphoreHandle_t modemMutex = NULL;
SemaphoreHandle_t taskSuspendMutex = NULL;

//Task Priorities
#define NETWORK_TASK_PRIORITY 3
#define OTA_TASK_PRIORITY 4

//Task Stack Sizes
#define NETWORK_TASK_STACK 8192
#define OTA_TASK_STACK 16384

// Counters
uint8_t modemInitRetries = 0;
uint8_t gprsRetries = 0;
uint8_t gsmErrorCount = 0;
const uint8_t MAX_MODEM_INIT_RETRIES = 3;
const uint8_t MAX_GPRS_RETRIES = 3;
const uint8_t MAX_GSM_ERRORS = 3;

// Objects
TinyGsm modem(SerialAT);
TinyGsmClient mqttClient(modem);
PubSubClient mqtt(mqttClient);

// ==================== Function Prototypes ====================
bool powerCycleModem();
bool initializeModem();
bool connectToGPRS();
bool connectToMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishHeartbeat();
bool checkForNewFirmware();
bool performOTAUpdate();

void networkTask(void* parameter);
void otaTask(void* parameter);

void suspendAllTasks();
void resumeAllTasks();
void createOTATask();
void deleteOTATask();

void GSM_setup() {
    // Initialize modem pins
    pinMode(MODEM_PWR, OUTPUT);
    digitalWrite(MODEM_PWR, LOW);

    // Start SIM7600 serial
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(1000);

      // Create mutexes
    mqttMutex = xSemaphoreCreateMutex();
    modemMutex = xSemaphoreCreateMutex();
    taskSuspendMutex = xSemaphoreCreateMutex();


    xTaskCreatePinnedToCore(networkTask, "NetworkTask", NETWORK_TASK_STACK, NULL, NETWORK_TASK_PRIORITY, &networkTaskHandle, 1);
}

// ==================== Network Functions ====================
bool powerCycleModem() {
    SerialMon.println("NetworkTask: Power cycling modem...");
    
    digitalWrite(MODEM_PWR, LOW);
    delay(1200);
    digitalWrite(MODEM_PWR, HIGH);
    delay(5000);  // Longer delay for SIM7600 to boot
    
    // Test with AT command
    SerialAT.println("AT");
    delay(100);
    
    String response = SerialAT.readString();
    if (response.indexOf("OK") == -1) {
        SerialMon.println("NetworkTask: Modem AT test failed");
        return false;
    }
    
    SerialMon.println("NetworkTask: Modem powered on successfully");
    return true;
}

bool initializeModem() {
    sendLedCommand(LED_GSM_INIT);
    SerialMon.println("NetworkTask: Initializing modem...");
    
    if (!modem.restart()) {
        SerialMon.println("NetworkTask: Modem restart failed");
        return false;
    }
    
    String modemInfo = modem.getModemInfo();
    SerialMon.print("NetworkTask: Modem Info: ");
    SerialMon.println(modemInfo);
    
    // Wait for network registration
    SerialMon.println("NetworkTask: Waiting for network...");
    
    if (!modem.waitForNetwork(180000)) {  // 3 minute timeout
        SerialMon.println("NetworkTask: Network registration timeout");
        return false;
    }
    
    SerialMon.println("NetworkTask: Network registered");
    return true;
}

bool connectToGPRS() {
    sendLedCommand(LED_CONNECTING);
    SerialMon.println("NetworkTask: Connecting to GPRS...");
    
    // Disconnect first if connected
    if (modem.isGprsConnected()) {
        modem.gprsDisconnect();
        delay(1000);
    }
    
    if (!modem.gprsConnect(apn, user, pass)) {
        SerialMon.println("NetworkTask: GPRS connection failed");
        return false;
    }
    
    char payload[80];

    SerialMon.printf("GPRS connected - IP: %s, Signal: %d\n",
        modem.getLocalIP().c_str(),
        modem.getSignalQuality());
    
    return true;
}

String mqttStateToText(int state) {
    switch (state) {
        case 0:   return "Connected";
        case -1:  return "Connection Timeout";
        case -2:  return "Connection Lost";
        case -3:  return "Connect Failed";
        case -4:  return "Disconnected";
        case -5:  return "Bad Protocol";
        case -6:  return "Bad Client ID";
        case -7:  return "Unavailable";
        case -8:  return "Bad Credentials";
        case -9:  return "Unauthorized";
        default:  return "Unknown";
    }
}

// Attempt MQTT connection with fallback
bool connectToMQTT() {
    sendLedCommand(LED_MQTT_CONNECTING);
    SerialMon.println("NetworkTask: Connecting to MQTT...");

    String clientId = "GMS-" + DEVICE_ID;
    bool connected = mqtt.connect(clientId.c_str(), mqttUser, mqttPass);

    // Retry once if first attempt fails
    if (!connected) {
        connected = mqtt.connect(clientId.c_str(), mqttUser, mqttPass);
    }
    // for(int i = 0; i < 5 && !connected; i++) {
    //     SerialMon.printf("NetworkTask: MQTT initial connect retry %d\n", i+1);
    //     if (!connected) {
    //         connected = mqtt.connect(clientId.c_str(), mqttUser, mqttPass);
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(500));
    // }

    if (connected) {
        SerialMon.println("NetworkTask: MQTT connected");
        String fullSubTopic = subTopic + DEVICE_ID;
        mqtt.subscribe(fullSubTopic.c_str());
        SerialMon.printf("NetworkTask: Subscribed to: %s\n", fullSubTopic.c_str());
    } else {
        Serial.printf("MQTT Connect failed (%d %s)\n", 
                        mqtt.state(), mqttStateToText(mqtt.state()).c_str());
    }

    return connected;
}

void networkTask(void* parameter) {
    // MQTT configuration
    mqtt.setServer(broker, 1883);
    mqtt.setKeepAlive(30);
    // mqtt.setSocketTimeout(10);
    mqtt.setCallback(mqttCallback);

    SerialMon.println("Network Task started");
    sendLedCommand(LED_OFFLINE);

    // Persistent state variables
    unsigned long lastSuccessfulOperation = 0;
    uint8_t consecutiveMqttFailures = 0;
    const uint8_t MAX_CONSECUTIVE_MQTT_FAILURES = 10;

    while (1) {
        // Skip if OTA is in progress
        if (otaInProgress) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Reset connection flags
        gprsConnected = false;
        mqttConnected = false;
        deviceOnline = false;
        consecutiveMqttFailures = 0;

        // ------------------------------------------------------------------
        // PHASE 1: Modem Initialization
        // ------------------------------------------------------------------
        SerialMon.println("\n=== NETWORK: Starting connection sequence ===");

        if (!modemWasInitialized || (millis() - lastSuccessfulOperation > 300000)) {
            SerialMon.println("NetworkTask: Initial modem power cycle...");
            if (!powerCycleModem()) {
                SerialMon.println("NetworkTask: Modem power cycle failed! ESP restarting...");
                delay(2000);
            }
        }

        // Initialize modem with retries
        int modemInitRetries = 0;
        while (!initializeModem() && modemInitRetries < MAX_MODEM_INIT_RETRIES) {
            modemInitRetries++;
            SerialMon.printf("NetworkTask: Modem init failed, retry %d/%d\n",
                            modemInitRetries, MAX_MODEM_INIT_RETRIES);
            if (modemInitRetries == 1) {
                SerialMon.println("NetworkTask: Trying modem soft reset...");
                modem.restart();
                delay(3000);
            }
            delay(5000);
        }

        if (modemInitRetries >= MAX_MODEM_INIT_RETRIES) {
            SerialMon.println("NetworkTask: Max modem init retries reached! ESP restarting...");
            delay(2000);
            ESP.restart();
        }

        modemWasInitialized = true;
        SerialMon.println("NetworkTask: Modem initialized successfully");

        // ------------------------------------------------------------------
        // PHASE 2: GPRS Connection (with retry & GSM error handling)
        // ------------------------------------------------------------------
        int gprsRetries = 0;
        int gsmErrorCount = 0;
        unsigned long connectionStartTime = millis();

        while (true) {
            // Timeout for the entire connection sequence
            if (millis() - connectionStartTime > 300000) {
                SerialMon.println("NetworkTask: Connection sequence timeout, restarting...");
                break;
            }

            // Attempt GPRS connection
            if (!connectToGPRS()) {
                gprsRetries++;
                SerialMon.printf("NetworkTask: GPRS connect failed, retry %d/%d\n",
                                gprsRetries, MAX_GPRS_RETRIES);

                if (gprsRetries >= MAX_GPRS_RETRIES) {
                    gsmErrorCount++;
                    SerialMon.printf("NetworkTask: GSM Error count: %d/%d\n",
                                    gsmErrorCount, MAX_GSM_ERRORS);

                    if (gsmErrorCount >= MAX_GSM_ERRORS) {
                        SerialMon.println("NetworkTask: Max GSM errors! ESP restarting...");
                        delay(2000);
                        ESP.restart();
                    }

                    // Wait before retrying GPRS again
                    unsigned long lastGsmErrorTime = millis();
                    while (millis() - lastGsmErrorTime < GSM_ERROR_RETRY_DELAY) {
                        sendLedCommand(LED_OFFLINE);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    gprsRetries = 0; // Reset retry counter after delay
                }
                delay(5000);
                continue; // Retry GPRS connection
            }

            // GPRS connected successfully
            gprsConnected = true;
            SerialMon.println("NetworkTask: GPRS connected");
            vTaskDelay(pdMS_TO_TICKS(500));
            lastSuccessfulOperation = millis();

            // ------------------------------------------------------------------
            // PHASE 3: MQTT Connection (with failure threshold)
            // ------------------------------------------------------------------
            while (gprsConnected) {
                if (!connectToMQTT()) {
                    consecutiveMqttFailures++;
                    SerialMon.printf("NetworkTask: MQTT connection failed %d/%d, rc=%d\n",
                                    consecutiveMqttFailures, MAX_CONSECUTIVE_MQTT_FAILURES,
                                    mqtt.state());

                    if (consecutiveMqttFailures >= MAX_CONSECUTIVE_MQTT_FAILURES) {
                        SerialMon.println("NetworkTask: Too many MQTT failures, restarting GPRS...");
                        modem.gprsDisconnect();
                        gprsConnected = false;
                        consecutiveMqttFailures = 0;
                        delay(2000);
                        break; // Exit MQTT loop and restart GPRS
                    }
                    delay(3000);
                    continue; // Retry MQTT
                }

                // MQTT connected successfully
                consecutiveMqttFailures = 0;
                mqttConnected = true;
                deviceOnline = true;
                sendLedCommand(LED_ONLINE);
                SerialMon.println("NetworkTask: Device ONLINE");
                lastSuccessfulOperation = millis();

                // ------------------------------------------------------------------
                // PHASE 4: Main Network Loop (keepalive, publish, connection checks)
                // ------------------------------------------------------------------
                unsigned long lastConnectionCheck = millis();
                unsigned long lastKeepalive = millis();
                const unsigned long CONNECTION_CHECK_INTERVAL = 10000;
                const unsigned long KEEPALIVE_INTERVAL = 30000;

                while (gprsConnected && mqttConnected) {
                    // sendLedCommand(LED_ONLINE);

                    // Periodic MQTT keepalive
                    if (millis() - lastKeepalive >= KEEPALIVE_INTERVAL) {
                        if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(1000))) {
                            mqtt.loop();
                            xSemaphoreGive(mqttMutex);
                        }
                        lastKeepalive = millis();
                    }

                    // Process incoming MQTT messages
                    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(100))) {
                        mqtt.loop();
                        xSemaphoreGive(mqttMutex);
                    }

                    // Process outgoing publish queue
                    MQTTMessage msg;
                    int queueSize = uxQueueMessagesWaiting(mqttPublishQueue);
                    for (int i = 0; i < queueSize; i++) {
                        if (xQueueReceive(mqttPublishQueue, &msg, 0) == pdTRUE) {
                            if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(500))) {
                                bool published = mqtt.publish(msg.topic, msg.payload);
                                xSemaphoreGive(mqttMutex);

                                if (published) {
                                    SerialMon.printf("NetworkTask: Published - %s: %s\n",
                                                        msg.topic, msg.payload);
                                } else {
                                    SerialMon.println("NetworkTask: Publish failed, requeuing");
                                    xQueueSendToFront(mqttPublishQueue, &msg, 0);
                                }
                            }
                        }
                    }

                    // Periodic connection checks
                    if (millis() - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
                        lastConnectionCheck = millis();

                        // Check GPRS
                        if (!modem.isGprsConnected()) {
                            SerialMon.println("NetworkTask: GPRS connection lost");
                            gprsConnected = false;
                            mqttConnected = false;
                            deviceOnline = false;
                            sendLedCommand(LED_OFFLINE);
                            break;
                        }

                        // Check MQTT
                        if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(500))) {
                            bool mqttAlive = mqtt.connected();
                            xSemaphoreGive(mqttMutex);
                            if (!mqttAlive) {
                                SerialMon.println("NetworkTask: MQTT connection lost");
                                mqttConnected = false;
                                deviceOnline = false;
                                break; // Break to MQTT reconnection loop
                            }
                        }

                        lastSuccessfulOperation = millis();
                    }

                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                // ------------------------------------------------------------------
                // PHASE 5: Connection Recovery
                // ------------------------------------------------------------------
                if (!gprsConnected) {
                    // GPRS lost – break to outer loop to restart from GPRS
                    mqttConnected = false;
                    deviceOnline = false;
                    sendLedCommand(LED_OFFLINE);
                    SerialMon.println("NetworkTask: GPRS lost, restarting connection...");
                    break; // Exit MQTT loop (goes to GPRS loop cleanup)
                }

                if (!mqttConnected) {
                    // Only MQTT lost – stay in GPRS loop and attempt MQTT reconnect
                    deviceOnline = false;
                    SerialMon.println("NetworkTask: MQTT lost, attempting reconnect...");

                    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(1000))) {
                        mqtt.disconnect();
                        xSemaphoreGive(mqttMutex);
                    }
                    delay(2000);
                    // Continue the inner while(gprsConnected) loop to retry MQTT
                }
            }

            // ------------------------------------------------------------------
            // Cleanup before restarting GPRS connection
            // ------------------------------------------------------------------
            if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(1000))) {
                mqtt.disconnect();
                xSemaphoreGive(mqttMutex);
            }

            if (modem.isGprsConnected()) {
                modem.gprsDisconnect();
                delay(1000);
            }

            gprsConnected = false;
            mqttConnected = false;
            deviceOnline = false;
            sendLedCommand(LED_OFFLINE);
            SerialMon.println("NetworkTask: Connection lost, restarting...");
            delay(3000);

            // Break the inner GPRS loop to restart from Phase 2 (GPRS connection)
            break;
        }

        // If we exit the GPRS loop, delay and then restart the entire sequence
        SerialMon.println("NetworkTask: Restarting connection sequence...");
        delay(5000);
    }
}

// ==================== OTA Task ====================
void otaTask(void* parameter) {
    SerialMon.println("OTA Task started");
    sendLedCommand(LED_OTA_IN_PROGRESS);

    suspendAllTasks();

    bool otaOk = false;

    if (checkForNewFirmware()) {
        if (xSemaphoreTake(modemMutex, portMAX_DELAY)) {
            otaOk = performOTAUpdate();
            xSemaphoreGive(modemMutex);
        }
    }

    if (otaOk) {
        Serial.println("[OTA] Success, rebooting");
        delay(1500);
        ESP.restart();
    }

    // OTA failed → clean GSM state
    modem.gprsDisconnect();
    vTaskDelay(pdMS_TO_TICKS(2000));

    resumeAllTasks();

    otaTaskHandle = NULL;
    vTaskDelete(NULL);
}

void createOTATask() {
    if (otaTaskHandle == NULL) {
        xTaskCreatePinnedToCore(otaTask, "OTATask", OTA_TASK_STACK, NULL, OTA_TASK_PRIORITY, &otaTaskHandle, 1);
        SerialMon.println("OTA Task created");
    }
}

void deleteOTATask() {
    if (otaTaskHandle != NULL) {
        vTaskDelete(otaTaskHandle);
        otaTaskHandle = NULL;
        SerialMon.println("OTA Task deleted");
    }
}


