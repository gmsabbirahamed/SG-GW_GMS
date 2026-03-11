/*
===     network     ===
network task workflow-
set mqtt
boot the modem (hard reset/off-on and test AT) ->if failed esp restart.
initialized modem (waitfornetwork and ..) -> if failed 3 times esp restart
connect to network (dis connect gprs & connect gprs with apn,user,pass)
if gprs is connected & mqtt is not connected -> try to connect/reconnect mqtt
if mqtt is connected && gprs is connected = online
if mqtt is not connected && gprs is not connected = offline
[if failed to connect gprs for 3 times that means gsm error -> wait for 10sec to retry gprs connection.
    esp restart after 3 gsm error]

===     main     ===
main task workflow-
send heartbeat in 1min interval
if device is online and data is availabe in queue -> send data to mqtt.
can be detect single press, double press, 3sec, 5sec, 10sec press.
[show real time led color status like during press and hold the button,, if complete 3sec it show solid green, then when complete 5sec the color change to solid blue, then when complete hold 10sec or more color change to solid red. but message will print after release the button..]

===      led status      ===
connecting network -> red blinking
offline -> solid red
online -> black
mqtt message received -> single blue blink
rf signal detect -> single white blink
heartbeat send -> single cyan blink
data send -> single green blink

===     rf sensor task      ===
read rf sensor and enqueue in queue - max 50 (if queue is full dequeue oldest data and enqueue new)

===     button task      ===
detect button press properly 
    ->show color status
    ->print message
    ->do action (updaet later)


===     special     ===
ota task will create dynamically ->when get command from mqtt msg, suspend/delete all other task & create ota task.
*/

/*
=== FINAL IOT GATEWAY FIRMWARE ===
ESP32 + SIM7600 GSM/GPRS + MQTT + OTA + RF Sensor + Button Control
Version: 1.0.0
Date: 2024-03-26
Author: Sabbir Ahamed
*/

// This code is designed to run on an ESP32 device with a SIM7600 GSM module.
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// Libraries required for GSM, MQTT, and ESP-NOW functionality
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Update.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h>
#include <FastLED.h>
#include <RCSwitch.h>
#include <Bounce2.h>

RCSwitch rfSwitch = RCSwitch();

Preferences preferences;

// ==================== Configuration ====================
#define WORK_PACKAGE "1101"
#define DEVICE_TYPE "00"
#define DEVICE_CODE_UPLOAD_DATE "250803"
#define DEVICE_SERIAL_ID "9999"

#define UNIQUE_DEVICE_ID WORK_PACKAGE DEVICE_TYPE DEVICE_CODE_UPLOAD_DATE DEVICE_SERIAL_ID

#define OTA 1
#define HW_VERSION "1.0.0"
#define FW_VERSION "1.0.2"
#define OTA_DATE "260101"

String DEVICE_ID = "";
String MAC_FALLBACK_ID = "";

// Serial and SIM-A7670 pin config
#define SerialMon Serial
#define SerialAT Serial1
#define MODEM_TX 17
#define MODEM_RX 16
#define MODEM_PWR 15
#define ONE_WIRE_BUS 19
#define HORN_PIN 25

// ==================== Button Configuration ====================
#define BUTTON_PIN 35
#define BUTTON_DEBOUNCE_INTERVAL 50  // 50ms debounce
#define DOUBLE_PRESS_TIMEOUT_MS 500  // Changed name to avoid conflict
#define HOLD_CHECK_INTERVAL 100       // Check hold every 100ms

// Button states for state machine
enum ButtonState {
    BUTTON_STATE_IDLE,
    BUTTON_STATE_PRESSED,
    BUTTON_STATE_HOLD_CHECK,
    BUTTON_STATE_HOLDING,
    BUTTON_STATE_RELEASED
};

// Button variables
Bounce2::Button button = Bounce2::Button();
volatile ButtonState currentButtonState = BUTTON_STATE_IDLE;
volatile unsigned long buttonPressStartTime = 0;
volatile unsigned long buttonReleaseTime = 0;
volatile unsigned int pressCount = 0;
volatile bool buttonStateChanged = false;

// For ISR handling
volatile bool buttonInterruptFlag = false;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;




// LED configuration
#define LED_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// RF Sensor pin
#define RF_SENSOR_PIN 26

// Configuration
const char apn[] = "internet";
const char user[] = "";
const char pass[] = "";

// MQTT Configuration
const char* broker = "broker2.dma-bd.com";
const char* mqttUser = "broker2";
const char* mqttPass = "Secret!@#$1234";

// OTA URLs
const char* otaHost = "ota.gmsabbirahamed.com";
const int otaPort = 80;
const char* versionPath = "/esp32/version.txt";
const char* firmwarePath = "/esp32/firmware/blink/firmware.bin";

// ==================== FreeRTOS Configuration ====================
// Task handles
TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t rfSensorTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;

// Queue handles
QueueHandle_t rfDataQueue = NULL;
QueueHandle_t ledCommandQueue = NULL;
QueueHandle_t mqttPublishQueue = NULL;
QueueHandle_t buttonEventQueue = NULL;

// Semaphore handles
SemaphoreHandle_t mqttMutex = NULL;
SemaphoreHandle_t modemMutex = NULL;
SemaphoreHandle_t taskSuspendMutex = NULL;

// Task priorities
#define MAIN_TASK_PRIORITY 2
#define NETWORK_TASK_PRIORITY 3
#define BUTTON_TASK_PRIORITY 1
#define RF_SENSOR_TASK_PRIORITY 2
#define LED_TASK_PRIORITY 1
#define OTA_TASK_PRIORITY 4

// Task stack sizes
#define MAIN_TASK_STACK 4096
#define NETWORK_TASK_STACK 8192
#define BUTTON_TASK_STACK 2048
#define RF_SENSOR_TASK_STACK 2048
#define LED_TASK_STACK 2048
#define OTA_TASK_STACK 16384

// Queue sizes
#define RF_DATA_QUEUE_SIZE 50
#define LED_CMD_QUEUE_SIZE 10
#define MQTT_PUB_QUEUE_SIZE 20
// #define BUTTON_EVENT_QUEUE_SIZE 5

// ==================== Data Structures ====================
enum LedCommandType {
    LED_OFFLINE,
    LED_CONNECTING,
    LED_ONLINE,
    LED_RF_DETECT,
    LED_HEARTBEAT,
    LED_PUBLISH_RF,
    LED_MQTT_RECEIVE,
    LED_BUTTON_PRESS,
    LED_BUTTON_HOLD_3SEC,
    LED_BUTTON_HOLD_5SEC,
    LED_BUTTON_HOLD_10SEC,
    LED_OTA_IN_PROGRESS
};

enum ButtonEventType {
    BUTTON_SINGLE_PRESS,
    BUTTON_DOUBLE_PRESS,
    BUTTON_HOLD_3SEC,
    BUTTON_HOLD_5SEC,
    BUTTON_HOLD_10SEC
};

struct LedCommand {
    LedCommandType type;
    uint8_t brightness;
};

struct RFData {
    uint32_t timestamp;
    uint32_t rf_value;
    uint8_t rf_length;
    uint8_t sensor_id;

};

struct MQTTMessage {
    char topic[50];
    char payload[100];
};

struct ButtonEvent {
    ButtonEventType type;
    unsigned long press_duration;
};

// ==================== Global Variables ====================
const char* pubTopic = "DMA/SG/PUB";
const char* subTopic = "DMA/SG/SUB";
const char* hbTopic = "DMA/SG/HB";

// System State
volatile bool mqttConnected = false;
volatile bool gprsConnected = false;
volatile bool deviceOnline = false;
volatile bool otaInProgress = false;
volatile bool tasksSuspended = false;

// Counters
uint8_t modemInitRetries = 0;
uint8_t gprsRetries = 0;
uint8_t gsmErrorCount = 0;
const uint8_t MAX_MODEM_INIT_RETRIES = 3;
const uint8_t MAX_GPRS_RETRIES = 3;
const uint8_t MAX_GSM_ERRORS = 3;

// Timing Configuration
const unsigned long HEARTBEAT_INTERVAL = 60000UL;
const unsigned long GSM_ERROR_RETRY_DELAY = 10000UL;
unsigned long lastHeartbeat = 0;
unsigned long lastGsmErrorTime = 0;
unsigned long buttonHoldStart = 0;
bool buttonHolding = false;

// Button variables
// volatile bool buttonPressed = false;
// volatile unsigned long buttonPressTime = 0;
// volatile unsigned long lastButtonPressTime = 0;
// volatile uint8_t buttonPressCount = 0;
// const unsigned long DOUBLE_PRESS_TIMEOUT = 500;

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
void sendLedCommand(LedCommandType type);
void sendButtonEvent(ButtonEventType type, unsigned long duration);
bool checkForNewFirmware();
bool performOTAUpdate();

// Task functions
void mainTask(void* parameter);
void networkTask(void* parameter);
void buttonTask(void* parameter);
void rfSensorTask(void* parameter);
void ledTask(void* parameter);
void otaTask(void* parameter);

// Helper functions
String getMACDeviceID();
void IRAM_ATTR buttonISR();
void suspendAllTasks();
void resumeAllTasks();
void createOTATask();
void deleteOTATask();

// ==================== Setup ====================
void setup() {
    SerialMon.begin(115200);
    delay(100);

    SerialMon.println("\n===================================");
    SerialMon.println("  ==      IoT GATEWAY FIRMWARE     ==");
    SerialMon.printf ("  ==       FW Version %          ==");
    SerialMon.printf ("  ==       HW Version 1.0.1          ==");
    SerialMon.println("  ==        Hello! Sabbir          ==");
    SerialMon.println("  ===================================\n");
    
    // Initialize LED hardware
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Black;
    FastLED.show();
    
    // Initialize button
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize RF sensor pin
    pinMode(RF_SENSOR_PIN, INPUT);
    pinMode(HORN_PIN, OUTPUT);

    digitalWrite(HORN_PIN, LOW);
    
    // Get MAC address for fallback ID
    MAC_FALLBACK_ID = getMACDeviceID();
    SerialMon.println("MAC Fallback ID: " + MAC_FALLBACK_ID);

    // Read or set Device ID
#if OTA
    preferences.begin("device", true);
    DEVICE_ID = preferences.getString("DID", MAC_FALLBACK_ID);
    preferences.end();
    SerialMon.println("[OTA MODE] Read-only mode: DEVICE_ID = " + DEVICE_ID);
#else
    preferences.begin("device", false);
    DEVICE_ID = preferences.getString("DID", "");

    if (DEVICE_ID.isEmpty() || DEVICE_ID != UNIQUE_DEVICE_ID) {
        SerialMon.println("[NORMAL MODE] Invalid or missing ID. Writing new ID...");
        DEVICE_ID = UNIQUE_DEVICE_ID;
        preferences.putString("DID", DEVICE_ID);
    }
    preferences.end();
#endif

    SerialMon.println("Final Device ID: " + DEVICE_ID);
    SerialMon.println("=================================\n");

    // Initialize modem pins
    pinMode(MODEM_PWR, OUTPUT);
    digitalWrite(MODEM_PWR, LOW);

    // Start SIM7600 serial
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(1000);

    // Increase HTTP buffer for OTA
    /*
    modem.sendAT("+SHTTPCFG=\"responseheader\",1");
    if (modem.waitResponse(1000L) != 1) {
        SerialMon.println("Failed to configure HTTP");
    }
    */

    // Create FreeRTOS objects
    rfDataQueue = xQueueCreate(RF_DATA_QUEUE_SIZE, sizeof(RFData));
    ledCommandQueue = xQueueCreate(LED_CMD_QUEUE_SIZE, sizeof(LedCommand));
    mqttPublishQueue = xQueueCreate(MQTT_PUB_QUEUE_SIZE, sizeof(MQTTMessage));
    // buttonEventQueue = xQueueCreate(BUTTON_EVENT_QUEUE_SIZE, sizeof(ButtonEvent));
    
    // Create mutexes
    mqttMutex = xSemaphoreCreateMutex();
    modemMutex = xSemaphoreCreateMutex();
    taskSuspendMutex = xSemaphoreCreateMutex();

    // Set initial LED state
    // sendLedCommand(LED_CONNECTING);
    sendLedCommand(LED_OFFLINE);
    
    // Attach button interrupt
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

    // Create tasks
    xTaskCreatePinnedToCore(mainTask, "MainTask", MAIN_TASK_STACK, NULL, MAIN_TASK_PRIORITY, &mainTaskHandle, 1);
    xTaskCreatePinnedToCore(networkTask, "NetworkTask", NETWORK_TASK_STACK, NULL, NETWORK_TASK_PRIORITY, &networkTaskHandle, 1);
    xTaskCreatePinnedToCore(buttonTask, "ButtonTask", BUTTON_TASK_STACK, NULL, BUTTON_TASK_PRIORITY, &buttonTaskHandle, 0);
    xTaskCreatePinnedToCore(rfSensorTask, "RFSensorTask", RF_SENSOR_TASK_STACK, NULL, RF_SENSOR_TASK_PRIORITY, &rfSensorTaskHandle, 0);
    xTaskCreatePinnedToCore(ledTask, "LedTask", LED_TASK_STACK, NULL, LED_TASK_PRIORITY, &ledTaskHandle, 0);
    
    SerialMon.println("All tasks created successfully");
}

void loop() {
    // FreeRTOS scheduler takes over - nothing to do here
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ==================== Task Implementations ====================
void mainTask(void* parameter) {
    unsigned long lastHeartbeatTime = 0;
    RFData rfData;
    MQTTMessage mqttMsg;
    ButtonEvent btnEvent;
    
    SerialMon.println("Main Task started");
    
    while (1) {
        if (otaInProgress) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Send heartbeat every minute
        if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
            if (deviceOnline) {
                publishHeartbeat();
                sendLedCommand(LED_HEARTBEAT);
                lastHeartbeatTime = millis();
            }
        }
        
        // Check for RF data in queue and publish if online
        if (deviceOnline && uxQueueMessagesWaiting(rfDataQueue) > 0) {
            if (xQueueReceive(rfDataQueue, &rfData, 0) == pdTRUE) {
                // Create MQTT message
                snprintf(mqttMsg.topic, sizeof(mqttMsg.topic), "%s", pubTopic);
                // snprintf(mqttMsg.payload, sizeof(mqttMsg.payload), 
                //          "{\"device\":\"%s\",\"type\":\"rf\",\"value\":%lu,\"timestamp\":%lu}",
                //          DEVICE_ID.c_str(), rfData.rf_value, rfData.rf_length);
                String payload = DEVICE_ID + "," + rfData.rf_value + "," + rfData.rf_length ;
                Serial.println(payload);
                // mqttMsg.payload = payload.c_str();
                snprintf(mqttMsg.payload, sizeof(mqttMsg.payload), "%s", payload.c_str());
                // mqtt.publish(mqttMsg.topic, payload.c_str());

                // Send to MQTT publish queue
                if (xQueueSend(mqttPublishQueue, &mqttMsg, pdMS_TO_TICKS(100)) == pdTRUE) {
                    sendLedCommand(LED_PUBLISH_RF);
                    SerialMon.printf("MainTask: RF data queued - Value: %lu\n", rfData.rf_value);
                }
            }
        }
        
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void networkTask(void* parameter) {
    // Configure MQTT
    mqtt.setServer(broker, 1883);
    mqtt.setKeepAlive(30);
    mqtt.setSocketTimeout(20);
    mqtt.setCallback(mqttCallback);
    
    SerialMon.println("Network Task started");
    // sendLedCommand(LED_CONNECTING);
    
    // State variables for connection management
    unsigned long lastSuccessfulOperation = 0;
    unsigned long connectionStartTime = 0;
    bool modemWasInitialized = false;
    uint8_t consecutiveMqttFailures = 0;
    const uint8_t MAX_CONSECUTIVE_MQTT_FAILURES = 5;
    
    while (1) {
        if (otaInProgress) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Reset connection state
        gprsConnected = false;
        mqttConnected = false;
        deviceOnline = false;
        consecutiveMqttFailures = 0;
        
        // ========== PHASE 1: MODEM INITIALIZATION ==========
        SerialMon.println("\n=== NETWORK: Starting connection sequence ===");
        sendLedCommand(LED_CONNECTING);
        
        // Only power cycle if modem was never initialized or has been stuck
        if (!modemWasInitialized || (millis() - lastSuccessfulOperation > 300000)) { // 5 minutes
            SerialMon.println("NetworkTask: Initial modem power cycle...");
            if (!powerCycleModem()) {
                SerialMon.println("NetworkTask: Modem power cycle failed! ESP restarting...");
                delay(2000);
                ESP.restart();
            }
        }
        
        // Initialize modem (with retries)
        modemInitRetries = 0;
        while (!initializeModem() && modemInitRetries < MAX_MODEM_INIT_RETRIES) {
            modemInitRetries++;
            SerialMon.printf("NetworkTask: Modem init failed, retry %d/%d\n", 
                           modemInitRetries, MAX_MODEM_INIT_RETRIES);
            
            // Try soft reset after first failure
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
        
        // ========== PHASE 2: GPRS CONNECTION ==========
        gprsRetries = 0;
        gsmErrorCount = 0;
        connectionStartTime = millis();
        
        while (1) {
            // Check if we should restart the whole sequence
            if (millis() - connectionStartTime > 300000) { // 5 minutes timeout
                SerialMon.println("NetworkTask: Connection sequence timeout, restarting...");
                break;
            }
            
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
                    
                    // Wait before retrying
                    lastGsmErrorTime = millis();
                    while (millis() - lastGsmErrorTime < GSM_ERROR_RETRY_DELAY) {
                        sendLedCommand(LED_OFFLINE);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    gprsRetries = 0;
                }
                delay(5000);
                continue;
            }
            
            // GPRS connected successfully
            gprsConnected = true;
            sendLedCommand(LED_CONNECTING);
            SerialMon.println("NetworkTask: GPRS connected");
            vTaskDelay(pdMS_TO_TICKS(500));
            lastSuccessfulOperation = millis();
            
            // ========== PHASE 3: MQTT CONNECTION ==========
            while (gprsConnected) {
                // Try to connect to MQTT
                if (!connectToMQTT()) {
                    consecutiveMqttFailures++;
                    SerialMon.printf("NetworkTask: MQTT connection failed %d/%d, rc=%d\n",
                                   consecutiveMqttFailures, MAX_CONSECUTIVE_MQTT_FAILURES, mqtt.state());
                    
                    if (consecutiveMqttFailures >= MAX_CONSECUTIVE_MQTT_FAILURES) {
                        SerialMon.println("NetworkTask: Too many MQTT failures, restarting GPRS...");
                        modem.gprsDisconnect();
                        gprsConnected = false;
                        consecutiveMqttFailures = 0;
                        sendLedCommand(LED_CONNECTING);
                        delay(2000);
                        break; // Go back to GPRS connection loop
                    }
                    // Wait before retrying MQTT
                    delay(3000);
                    continue;
                }
                
                // MQTT connected successfully
                consecutiveMqttFailures = 0;
                mqttConnected = true;
                deviceOnline = true;
                sendLedCommand(LED_ONLINE);
                SerialMon.println("NetworkTask: Device ONLINE");
                lastSuccessfulOperation = millis();
                
                // ========== PHASE 4: MAIN NETWORK LOOP ==========
                unsigned long lastConnectionCheck = millis();
                unsigned long lastKeepalive = millis();
                const unsigned long CONNECTION_CHECK_INTERVAL = 10000; // 10 seconds
                const unsigned long KEEPALIVE_INTERVAL = 30000; // 30 seconds
                
                while (gprsConnected && mqttConnected) {
                    // Send periodic MQTT keepalive
                    if (millis() - lastKeepalive >= KEEPALIVE_INTERVAL) {
                        if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(1000))) {
                            mqtt.loop(); // This handles keepalive
                            xSemaphoreGive(mqttMutex);
                        }
                        lastKeepalive = millis();
                    }
                    
                    // Process MQTT messages
                    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(100))) {
                        mqtt.loop();
                        xSemaphoreGive(mqttMutex);
                    }
                    
                    // Process MQTT publish queue
                    MQTTMessage msg;
                    int publishQueueItems = uxQueueMessagesWaiting(mqttPublishQueue);
                    
                    for (int i = 0; i < publishQueueItems; i++) {
                        if (xQueueReceive(mqttPublishQueue, &msg, 0) == pdTRUE) {
                            if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(500))) {
                                bool published = mqtt.publish(msg.topic, msg.payload);
                                xSemaphoreGive(mqttMutex);
                                
                                if (published) {
                                    SerialMon.printf("NetworkTask: Published - %s: %s\n", 
                                                   msg.topic, msg.payload);
                                } else {
                                    // Re-queue if failed
                                    SerialMon.println("NetworkTask: Publish failed, requeuing");
                                    xQueueSendToFront(mqttPublishQueue, &msg, 0);
                                }
                            }
                        }
                    }
                    
                    // Check connections periodically (not every loop)
                    if (millis() - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
                        lastConnectionCheck = millis();
                        
                        // Check GPRS connection
                        bool gprsAlive = modem.isGprsConnected();
                        if (!gprsAlive) {
                            SerialMon.println("NetworkTask: GPRS connection lost");
                            gprsConnected = false;
                            mqttConnected = false;
                            deviceOnline = false;
                            sendLedCommand(LED_OFFLINE);
                            break;
                        }
                        
                        // Check MQTT connection
                        if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(500))) {
                            bool mqttAlive = mqtt.connected();
                            xSemaphoreGive(mqttMutex);
                            
                            if (!mqttAlive) {
                                SerialMon.println("NetworkTask: MQTT connection lost");
                                mqttConnected = false;
                                deviceOnline = false;
                                sendLedCommand(LED_CONNECTING);
                                break;
                            }
                        }
                        
                        lastSuccessfulOperation = millis();
                    }
                    
                    // Small delay to prevent CPU hogging
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
                // ========== PHASE 5: CONNECTION RECOVERY ==========
                if (!gprsConnected) {
                    // GPRS was lost, break to outer loop to restart
                    mqttConnected = false;
                    deviceOnline = false;
                    sendLedCommand(LED_OFFLINE);
                    SerialMon.println("NetworkTask: GPRS lost, restarting connection...");
                    break;
                }
                
                if (!mqttConnected) {
                    // Only MQTT was lost, try to reconnect
                    deviceOnline = false;
                    sendLedCommand(LED_CONNECTING);
                    SerialMon.println("NetworkTask: MQTT lost, attempting reconnect...");
                    
                    // Clean up MQTT
                    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(1000))) {
                        mqtt.disconnect();
                        xSemaphoreGive(mqttMutex);
                    }
                    
                    // Small delay before reconnecting
                    delay(2000);
                    
                    // Continue to MQTT connection loop
                    continue;
                }
            }
            
            // Cleanup before restarting GPRS connection
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
            
            // Break to restart the whole connection sequence
            break;
        }
        
        // If we get here, connection sequence failed, restart
        SerialMon.println("NetworkTask: Restarting connection sequence...");
        delay(5000);
    }
}



void buttonTask(void* parameter) {
    // Initialize button with Bounce2
    button.attach(BUTTON_PIN, INPUT_PULLUP);
    button.interval(BUTTON_DEBOUNCE_INTERVAL);
    button.setPressedState(LOW);  // Button is active low
    
    unsigned long pressStartTime = 0;
    bool isPressing = false;
    int currentHoldThreshold = 0;  // 0=none, 3=3sec, 5=5sec, 10=10sec
    int lastPrintedThreshold = 0;  // To avoid repeated messages
    
    SerialMon.println("Button Task started");
    
    while (1) {
        if (otaInProgress) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Handle ISR flag
        if (buttonInterruptFlag) {
            portENTER_CRITICAL(&buttonMux);
            buttonInterruptFlag = false;
            portEXIT_CRITICAL(&buttonMux);
            button.update();
        } else {
            button.update();
        }
        
        unsigned long currentTime = millis();
        
        // ========== BUTTON PRESS DETECTION ==========
        if (button.pressed()) {
            // Button just pressed
            pressStartTime = currentTime;
            isPressing = true;
            currentHoldThreshold = 0;
            lastPrintedThreshold = 0;
            
            SerialMon.println("Button pressed - showing yellow blink");
            sendLedCommand(LED_BUTTON_PRESS);  // Yellow flash
        }
        
        // ========== VISUAL FEEDBACK DURING HOLD ==========
        if (isPressing) {
            unsigned long holdDuration = currentTime - pressStartTime;
            
            // Determine current threshold based on hold duration
            int newThreshold = 0;
            if (holdDuration >= 10000) {
                newThreshold = 10;
            } else if (holdDuration >= 5000) {
                newThreshold = 5;
            } else if (holdDuration >= 3000) {
                newThreshold = 3;
            }
            
            // Update LED and print message ONLY when threshold changes
            if (newThreshold != currentHoldThreshold) {
                currentHoldThreshold = newThreshold;
                
                switch (currentHoldThreshold) {
                    case 3:
                        sendLedCommand(LED_BUTTON_HOLD_3SEC);  // Solid cyan
                        if (lastPrintedThreshold != 3) {
                            SerialMon.println("Hold: 3 seconds reached - LED cyan");
                            lastPrintedThreshold = 3;
                        }
                        break;
                    case 5:
                        sendLedCommand(LED_BUTTON_HOLD_5SEC);  // Solid pink
                        if (lastPrintedThreshold != 5) {
                            SerialMon.println("Hold: 5 seconds reached - LED pink");
                            lastPrintedThreshold = 5;
                        }
                        break;
                    case 10:
                        sendLedCommand(LED_BUTTON_HOLD_10SEC);  // Solid purple
                        if (lastPrintedThreshold != 10) {
                            SerialMon.println("Hold: 10 seconds reached - LED purple");
                            lastPrintedThreshold = 10;
                        }
                        break;
                }
            }
        }
        
        // ========== BUTTON RELEASE DETECTION ==========
        if (button.released() && isPressing) {
            unsigned long holdDuration = currentTime - pressStartTime;
            isPressing = false;
            
            // Print appropriate message based on hold duration
            SerialMon.println("\n========== BUTTON EVENT ==========");
            
            if (holdDuration >= 10000) {
                SerialMon.println("10 SECOND HOLD DETECTED");
                SerialMon.printf("Hold duration: %lu ms\n", holdDuration);
                SerialMon.println("Final LED was: Purple");
                sendButtonEvent(BUTTON_HOLD_10SEC, holdDuration);
            }
            else if (holdDuration >= 5000) {
                SerialMon.println("5 SECOND HOLD DETECTED");
                SerialMon.printf("Hold duration: %lu ms\n", holdDuration);
                SerialMon.println("Final LED was: Pink");
                sendButtonEvent(BUTTON_HOLD_5SEC, holdDuration);
            }
            else if (holdDuration >= 3000) {
                SerialMon.println("3 SECOND HOLD DETECTED");
                SerialMon.printf("Hold duration: %lu ms\n", holdDuration);
                SerialMon.println("Final LED was: Cyan");
                sendButtonEvent(BUTTON_HOLD_3SEC, holdDuration);
            }
            else {
                SerialMon.println("SINGLE CLICK DETECTED");
                SerialMon.printf("Hold duration: %lu ms\n", holdDuration);
                SerialMon.println("LED: Yellow blink");
                sendButtonEvent(BUTTON_SINGLE_PRESS, holdDuration);
            }
            
            SerialMon.println("===================================\n");
            
            // Return LED to network state
            vTaskDelay(pdMS_TO_TICKS(300));  // Short delay to show the final color
            
            if (deviceOnline) {
                sendLedCommand(LED_ONLINE);  // Black (off)
                SerialMon.println("LED: Returning to ONLINE state (OFF)");
            } else if (gprsConnected) {
                sendLedCommand(LED_CONNECTING);  // Red blinking
                SerialMon.println("LED: Returning to CONNECTING state (red blink)");
            } else {
                sendLedCommand(LED_OFFLINE);  // Solid red
                SerialMon.println("LED: Returning to OFFLINE state (solid red)");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms check interval
    }
}

void rfSensorTask(void* parameter) {
    rfSwitch.enableReceive(RF_SENSOR_PIN);
    
    SerialMon.println("RF Sensor Task started");
    
    uint32_t lastRFValue = 0;
    unsigned long lastRFTime = 0;
    const unsigned long DEBOUNCE_TIME = 800;
    
    while (1) {
        if (otaInProgress) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        if (rfSwitch.available()) {
            uint32_t rfValue = rfSwitch.getReceivedValue();
            uint32_t rfLength = rfSwitch.getReceivedBitlength();
            rfSwitch.resetAvailable();
            
            // Debounce: Ignore same value within DEBOUNCE_TIME
            if (rfValue == lastRFValue && millis() - lastRFTime < DEBOUNCE_TIME) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            lastRFValue = rfValue;
            lastRFTime = millis();
            
            // Create RF data
            RFData rfData;
            rfData.timestamp = millis();
            rfData.rf_value = rfValue;
            rfData.rf_length = rfLength;
            rfData.sensor_id = 1;
            
            // Check if queue is full
            if (uxQueueMessagesWaiting(rfDataQueue) >= RF_DATA_QUEUE_SIZE) {
                // Remove oldest item
                RFData oldData;
                xQueueReceive(rfDataQueue, &oldData, 0);
                SerialMon.println("RFSensorTask: Queue full, removed oldest item");
            }
            
            // Add to queue
            if (xQueueSend(rfDataQueue, &rfData, 0) == pdTRUE) {
                sendLedCommand(LED_RF_DETECT);
                SerialMon.printf("RFSensorTask: RF detected - Value: %lu\n", rfValue);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ledTask(void* parameter) {
    unsigned long lastBlinkTime = 0;
    bool blinkState = false;
    LedCommand currentCommand;
    currentCommand.type = LED_CONNECTING;
    currentCommand.brightness = 100;
    
    SerialMon.println("LED Task started");
    
    while (1) {
        // Check for new LED commands
        LedCommand newCommand;
        if (xQueueReceive(ledCommandQueue, &newCommand, 0) == pdTRUE) {
            currentCommand = newCommand;
            lastBlinkTime = millis();
            blinkState = false;
        }
        
        // Execute LED command
        switch (currentCommand.type) {
            case LED_OFFLINE:
                leds[0] = CRGB::Red;
                break;
                
            case LED_CONNECTING:
                // Red blinking
                if (millis() - lastBlinkTime > 500) {
                    blinkState = !blinkState;
                    leds[0] = blinkState ? CRGB::Red : CRGB::Black;
                    lastBlinkTime = millis();
                }
                break;
                
            case LED_ONLINE:
                leds[0] = CRGB::Black;
                break;
                
            case LED_RF_DETECT:
                // Single white blink
                leds[0] = CRGB::White;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (gprsConnected) {
                    currentCommand.type = LED_CONNECTING;
                } else {
                    currentCommand.type = LED_OFFLINE;
                }
                break;
                
            case LED_HEARTBEAT:
                // Single cyan blink
                leds[0] = CRGB::Cyan;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (gprsConnected) {
                    currentCommand.type = LED_CONNECTING;
                } else {
                    currentCommand.type = LED_OFFLINE;
                }
                break;
                
            case LED_PUBLISH_RF:
                // Single green blink
                leds[0] = CRGB::Green;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (gprsConnected) {
                    currentCommand.type = LED_CONNECTING;
                } else {
                    currentCommand.type = LED_OFFLINE;
                }
                break;
                
            case LED_MQTT_RECEIVE:
                // Single blue blink
                leds[0] = CRGB::Blue;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (gprsConnected) {
                    currentCommand.type = LED_CONNECTING;
                } else {
                    currentCommand.type = LED_OFFLINE;
                }
                break;
                
            case LED_BUTTON_PRESS:
                // Yellow flash
                leds[0] = CRGB::Yellow;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(50));
                leds[0] = CRGB::Black;
                break;
                
            case LED_BUTTON_HOLD_3SEC:
                leds[0] = CRGB::LightCyan;
                break;
                
            case LED_BUTTON_HOLD_5SEC:
                leds[0] = CRGB::Blue;
                break;
                
            case LED_BUTTON_HOLD_10SEC:
                leds[0] = CRGB::Pink;
                break;
                
            case LED_OTA_IN_PROGRESS:
                // Purple blinking for OTA
                if (millis() - lastBlinkTime > 300) {
                    blinkState = !blinkState;
                    leds[0] = blinkState ? CRGB::Purple : CRGB::Black;
                    lastBlinkTime = millis();
                }
                break;
        }
        
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
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
    
    String ip = modem.getLocalIP();
    SerialMon.print("NetworkTask: GPRS connected, IP: ");
    SerialMon.println(ip);
    
    return true;
}

bool connectToMQTT() {
    SerialMon.println("NetworkTask: Connecting to MQTT...");
    
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(5000))) {
        
        // mqtt.disconnect();
        // mqttClient.stop();
        delay(500);
        // Generate unique client ID
        // String clientId = "GMS-" + DEVICE_ID + "-" + String(esp_random() & 0xFFFF);
        String clientId = "GMS-" + DEVICE_ID ;
        
        // Set shorter timeout for first attempt
        mqtt.setSocketTimeout(15);
        
        bool connected = mqtt.connect(clientId.c_str(), mqttUser, mqttPass);
        
        if (!connected) {
            // Try with different socket timeout
            mqtt.setSocketTimeout(30);
            connected = mqtt.connect(clientId.c_str(), mqttUser, mqttPass);
        }
        
        if (connected) {
            SerialMon.println("NetworkTask: MQTT connected");
            
            // Subscribe to topics
            mqtt.subscribe(subTopic);
            SerialMon.printf("NetworkTask: Subscribed to: %s\n", subTopic);
            
            // Send initialization message
            MQTTMessage initMsg;
            snprintf(initMsg.topic, sizeof(initMsg.topic), "%s", pubTopic);
            snprintf(initMsg.payload, sizeof(initMsg.payload), 
                     "{\"device\":\"%s\",\"type\":\"init\",\"status\":\"online\",\"version\":\"%s\",\"rssi\":%d}",
                     DEVICE_ID.c_str(), FW_VERSION, modem.getSignalQuality());
            
            // Send directly (not through queue) to ensure it goes out
            if (mqtt.publish(initMsg.topic, initMsg.payload)) {
                SerialMon.println("NetworkTask: Initialization message sent");
            } else {
                SerialMon.println("NetworkTask: Failed to send initialization message");
            }
            
            // Restore normal socket timeout
            mqtt.setSocketTimeout(30);
        } else {
            SerialMon.printf("NetworkTask: MQTT connection failed, rc=%d\n", mqtt.state());
            
            // Additional debug info
            if (mqtt.state() == -4) {
                SerialMon.println("NetworkTask: MQTT timeout - check broker address/port");
            }
        }
        
        xSemaphoreGive(mqttMutex);
        return connected;
    }
    
    return false;
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    message.trim();
    
    SerialMon.printf("NetworkTask: MQTT message - Topic: %s, Payload: %s\n", topic, message.c_str());
    sendLedCommand(LED_MQTT_RECEIVE);
    
    if (strcmp(topic, subTopic) == 0) {
        // Parse JSON or command
        if (message.indexOf("ping") != -1) {
            MQTTMessage response;
            snprintf(response.topic, sizeof(response.topic), "%s", pubTopic);
            snprintf(response.payload, sizeof(response.payload), "{\"device\":\"%s\",\"rssi\":\"%d\"}", DEVICE_ID.c_str(), modem.getSignalQuality());
            xQueueSend(mqttPublishQueue, &response, pdMS_TO_TICKS(100));
        }
        else if (message.indexOf("rssi") != -1) {
            int rssi = modem.getSignalQuality();
            MQTTMessage response;
            snprintf(response.topic, sizeof(response.topic), "%s", pubTopic);
            snprintf(response.payload, sizeof(response.payload), 
                     "{\"device\":\"%s\",\"rssi\":%d}", DEVICE_ID.c_str(), rssi);
            xQueueSend(mqttPublishQueue, &response, pdMS_TO_TICKS(100));
        }


        else if (message.indexOf("alarm off") != -1) {
            digitalWrite(HORN_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(300));
            digitalWrite(HORN_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(200));
            digitalWrite(HORN_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(300));
            digitalWrite(HORN_PIN, LOW);
        }
        else if (message.indexOf("alarm on") != -1) {
            digitalWrite(HORN_PIN, HIGH);
        }


        else if (message.indexOf("ota") != -1) {
            // Create OTA task
            createOTATask();
        }
        else if (message.indexOf("restart") != -1) {
            MQTTMessage response;
            snprintf(response.topic, sizeof(response.topic), "%s", pubTopic);
            snprintf(response.payload, sizeof(response.payload), 
                     "{\"device\":\"%s\",\"status\":\"restarting\"}", DEVICE_ID.c_str());
            xQueueSend(mqttPublishQueue, &response, pdMS_TO_TICKS(100));
            delay(1000);
            ESP.restart();
        }
    }
}

//1102032505280029,W:1,G:1,C:1,M:1,V:1.472
void publishHeartbeat() {
    MQTTMessage hbMsg;
    snprintf(hbMsg.topic, sizeof(hbMsg.topic), "%s", hbTopic);
    // snprintf(hbMsg.payload, sizeof(hbMsg.payload), 
    //          "{\"device\":\"%s\",\"type\":\"heartbeat\",\"hw\":\"%s\",\"fw\":\"%s\",\"uptime\":%lu}",
    //          DEVICE_ID.c_str(), HW_VERSION, FW_VERSION, millis() / 1000);

    String payload = DEVICE_ID + ",W:1,G:1,C:1,M:1,V:1.472" ;
                Serial.println(payload);
                // mqttMsg.payload = payload.c_str();
                snprintf(hbMsg.payload, sizeof(hbMsg.payload), "%s", payload.c_str());
    
    if (xQueueSend(mqttPublishQueue, &hbMsg, pdMS_TO_TICKS(100)) == pdTRUE) {
        SerialMon.println("MainTask: Heartbeat queued");
    }
}

// ==================== Helper Functions ====================
String getMACDeviceID() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

void IRAM_ATTR buttonISR() {
    portENTER_CRITICAL_ISR(&buttonMux);
    buttonInterruptFlag = true;
    portEXIT_CRITICAL_ISR(&buttonMux);
}

void sendLedCommand(LedCommandType type) {
    if (ledCommandQueue == NULL) return;
    
    LedCommand cmd;
    cmd.type = type;
    cmd.brightness = 100;
    xQueueSend(ledCommandQueue, &cmd, 0);
}

void sendButtonEvent(ButtonEventType type, unsigned long duration) {
    if (buttonEventQueue == NULL) return;
    
    ButtonEvent event;
    event.type = type;
    event.press_duration = duration;
    xQueueSend(buttonEventQueue, &event, 0);
}

void suspendAllTasks() {
    if (xSemaphoreTake(taskSuspendMutex, portMAX_DELAY)) {
        otaInProgress = true;
        tasksSuspended = true;
        
        vTaskSuspend(mainTaskHandle);
        vTaskSuspend(networkTaskHandle);
        vTaskSuspend(buttonTaskHandle);
        vTaskSuspend(rfSensorTaskHandle);
        // LED task continues to show OTA status
        
        xSemaphoreGive(taskSuspendMutex);
        SerialMon.println("All tasks suspended for OTA");
    }
}

void resumeAllTasks() {
    if (xSemaphoreTake(taskSuspendMutex, portMAX_DELAY)) {
        otaInProgress = false;
        tasksSuspended = false;
        
        vTaskResume(mainTaskHandle);
        vTaskResume(networkTaskHandle);
        vTaskResume(buttonTaskHandle);
        vTaskResume(rfSensorTaskHandle);
        
        xSemaphoreGive(taskSuspendMutex);
        SerialMon.println("All tasks resumed");
    }
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

// ==================== OTA Functions ====================
bool checkForNewFirmware() {
    SerialMon.println("OTA: Checking for new firmware...");
    
    // Implement your firmware version check logic here
    // This should compare current FW_VERSION with server version
    
    return true;  // Return true if new firmware is available
    // return false;  // Return true if new firmware is available
}

bool performOTAUpdate() {
    SerialMon.println("\n[OTA] Starting OTA update");
    
    // DON'T disconnect GSM here - OTA needs active connection!
    // Keep GPRS connected throughout OTA
    
    // Create a dedicated client for OTA
    TinyGsmClient otaClient(modem);
    
    unsigned long otaStartTime = millis();
    const unsigned long OTA_TOTAL_TIMEOUT = 300000; // 5 minutes total timeout
    
    // ========== STEP 1: CONNECT TO OTA SERVER ==========
    SerialMon.printf("[OTA] Connecting to %s:%d\n", otaHost, otaPort);
    
    int connectAttempts = 0;
    bool serverConnected = false;
    
    while (connectAttempts < 3 && !serverConnected) {
        connectAttempts++;
        SerialMon.printf("[OTA] Connection attempt %d/3\n", connectAttempts);
        
        if (otaClient.connect(otaHost, otaPort)) {
            serverConnected = true;
            SerialMon.println("[OTA] Connected to server");
        } else {
            SerialMon.println("[OTA] Connection failed");
            if (connectAttempts < 3) {
                delay(2000);
                
                // Check if GSM is still connected
                if (!modem.isGprsConnected()) {
                    SerialMon.println("[OTA] GPRS lost during OTA, aborting");
                    return false;
                }
            }
        }
    }
    
    if (!serverConnected) {
        SerialMon.println("[OTA] Failed to connect to server");
        otaClient.stop();
        return false;
    }
    
    // ========== STEP 2: SEND HTTP REQUEST ==========
    SerialMon.printf("[OTA] Requesting firmware: %s\n", firmwarePath);
    
    String request = String("GET ") + firmwarePath + " HTTP/1.1\r\n" +
                    "Host: " + otaHost + "\r\n" +
                    "User-Agent: ESP32-OTA\r\n" +
                    "Connection: keep-alive\r\n" +
                    "\r\n";
    
    otaClient.print(request);
    
    // ========== STEP 3: PARSE HTTP RESPONSE ==========
    unsigned long headerTimeout = millis();
    const unsigned long HEADER_TIMEOUT = 10000;
    bool headersComplete = false;
    int contentLength = 0;
    bool httpOK = false;
    
    SerialMon.println("[OTA] Waiting for HTTP response...");
    
    while (millis() - headerTimeout < HEADER_TIMEOUT && !headersComplete) {
        if (otaClient.available()) {
            String line = otaClient.readStringUntil('\n');
            line.trim();
            
            SerialMon.println("[OTA HDR] " + line);
            
            if (line.length() == 0) {
                // Empty line indicates end of headers
                headersComplete = true;
                break;
            }
            
            if (line.startsWith("HTTP/1.")) {
                if (line.indexOf("200") > 0) {
                    httpOK = true;
                }
            }
            
            if (line.startsWith("Content-Length:")) {
                contentLength = line.substring(15).toInt();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (!headersComplete) {
        SerialMon.println("[OTA] HTTP header timeout");
        otaClient.stop();
        return false;
    }
    
    if (!httpOK) {
        SerialMon.println("[OTA] HTTP not OK");
        otaClient.stop();
        return false;
    }
    
    if (contentLength <= 0) {
        SerialMon.println("[OTA] Invalid content length");
        otaClient.stop();
        return false;
    }
    
    SerialMon.printf("[OTA] Firmware size: %d bytes\n", contentLength);
    
    // ========== STEP 4: PREPARE UPDATE ==========
    if (!Update.begin(contentLength, U_FLASH)) {
        SerialMon.printf("[OTA] Update.begin failed: %s\n", Update.errorString());
        otaClient.stop();
        return false;
    }
    
    // ========== STEP 5: DOWNLOAD FIRMWARE ==========
    uint8_t buffer[512];
    size_t totalWritten = 0;
    unsigned long lastDataTime = millis();
    unsigned long lastProgressTime = millis();
    const unsigned long DATA_TIMEOUT = 60000; // 30 seconds timeout between data
    const unsigned long PROGRESS_INTERVAL = 1000; // Update progress every second
    
    SerialMon.println("[OTA] Starting firmware download...");
    
    while (totalWritten < contentLength) {
        // Check total timeout
        if (millis() - otaStartTime > OTA_TOTAL_TIMEOUT) {
            SerialMon.printf("[OTA] Total OTA timeout (%lu ms)\n", millis() - otaStartTime);
            Update.abort();
            otaClient.stop();
            return false;
        }
        
        // Check if GSM is still connected
        if (!modem.isGprsConnected()) {
            SerialMon.println("[OTA] GPRS connection lost during download");
            Update.abort();
            otaClient.stop();
            return false;
        }
        
        // Read available data
        if (otaClient.available()) {
            lastDataTime = millis();
            
            int bytesAvailable = otaClient.available();
            int toRead = min(min(bytesAvailable, (int)sizeof(buffer)), 
                           (int)(contentLength - totalWritten));
            
            if (toRead > 0) {
                int bytesRead = otaClient.read(buffer, toRead);
                
                if (bytesRead > 0) {
                    // Write to flash
                    if (Update.write(buffer, bytesRead) != bytesRead) {
                        SerialMon.printf("[OTA] Write error at byte %d\n", totalWritten);
                        Update.abort();
                        otaClient.stop();
                        return false;
                    }
                    
                    totalWritten += bytesRead;
                    
                    // Show progress
                    if (millis() - lastProgressTime > PROGRESS_INTERVAL) {
                        float percent = (100.0 * totalWritten) / contentLength;
                        SerialMon.printf("[OTA] Progress: %.1f%% (%d/%d bytes)\n", 
                                       percent, totalWritten, contentLength);
                        lastProgressTime = millis();
                    }
                }
            }
        } else {
            // No data available, small delay
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Check for data timeout
            if (millis() - lastDataTime > DATA_TIMEOUT) {
                SerialMon.printf("[OTA] Data timeout after %lu ms (at %d/%d bytes)\n", 
                               millis() - lastDataTime, totalWritten, contentLength);
                Update.abort();
                otaClient.stop();
                return false;
            }
        }
        
        // Feed watchdog
        esp_task_wdt_reset();
    }
    
    // ========== STEP 6: FINALIZE UPDATE ==========
    SerialMon.println("\n[OTA] Download complete, finalizing...");
    
    if (!Update.end()) {
        SerialMon.printf("[OTA] Update.end failed: %s\n", Update.errorString());
        otaClient.stop();
        return false;
    }
    
    if (!Update.isFinished()) {
        SerialMon.println("[OTA] Update not finished");
        otaClient.stop();
        return false;
    }
    
    SerialMon.println("[OTA] Firmware update successful!");
    otaClient.stop();
    
    // Wait a moment and restart
    SerialMon.println("[OTA] Restarting in 3 seconds...");
    delay(3000);
    
    ESP.restart();
    
    return true; // Never reaches here
}