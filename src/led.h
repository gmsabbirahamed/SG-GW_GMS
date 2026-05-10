#pragma once

#include <FastLED.h>
#include "config.h"

// LED configuration
#define LED_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

enum LedCommandType {
    LED_OFFLINE,
    LED_CONNECTING,
    LED_GSM_INIT,
    LED_MQTT_CONNECTING,
    LED_ONLINE,
    LED_RF_DETECT,
    LED_HEARTBEAT,
    LED_PUBLISH_EM,
    LED_MQTT_RECEIVE,
    LED_DISARMED,
    LED_BUTTON_PRESS,
    LED_BUTTON_HOLD_3SEC,
    LED_BUTTON_HOLD_5SEC,
    LED_BUTTON_HOLD_10SEC,
    LED_OTA_IN_PROGRESS,
    LED_PING_ACK
};

TaskHandle_t ledTaskHandle = NULL;
QueueHandle_t ledCommandQueue = NULL;

#define LED_TASK_PRIORITY 1
#define LED_CMD_QUEUE_SIZE 10
#define LED_TASK_STACK 2048

struct LedCommand {
    LedCommandType type;
    uint8_t brightness;
};

void sendLedCommand(LedCommandType type);
void ledTask(void* parameter);

void FastLED_setup(){
    // Initialize LED hardware
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Black;
    FastLED.show();

    ledCommandQueue = xQueueCreate(LED_CMD_QUEUE_SIZE, sizeof(LedCommand));
    sendLedCommand(LED_OFFLINE);
    xTaskCreatePinnedToCore(ledTask, "LedTask", LED_TASK_STACK, NULL, LED_TASK_PRIORITY, &ledTaskHandle, 0);
}

void sendLedCommand(LedCommandType type) {
    if (ledCommandQueue == NULL) return;
    
    LedCommand cmd;
    cmd.type = type;
    cmd.brightness = 100;
    xQueueSend(ledCommandQueue, &cmd, 0);
}

void ledTask(void* parameter) {
    unsigned long lastBlinkTime = 0;
    bool blinkState = false;
    LedCommand currentCommand;
    currentCommand.type = LED_OFFLINE;
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

            case LED_GSM_INIT:
            {
                //Rainbow animation
                // static uint8_t hue = 0;
                // leds[0] = CHSV(hue = hue+10, 255, currentCommand.brightness);
                // leds[0] = CHSV(hue = hue+5, 255, 255);
                if (millis() - lastBlinkTime > 500) {
                    blinkState = !blinkState;
                    leds[0] = blinkState ? CRGB::Red : CRGB::Black;
                    lastBlinkTime = millis();
                }
            }
            break;
                // Solid Purple
                // leds[0] = CRGB::Purple;
                // break;

            case LED_MQTT_CONNECTING:
                // Yellow blinking
                if (millis() - lastBlinkTime > 500) {
                    blinkState = !blinkState;
                    leds[0] = blinkState ? CRGB::Yellow : CRGB::Black;
                    lastBlinkTime = millis();
                }
                break;
                
            case LED_ONLINE:
                leds[0] = CRGB::Black;
                break;
                
            case LED_RF_DETECT:
                // Single white blink
                leds[0] = CRGB::Blue;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(250));
                leds[0] = CRGB::Black;
                FastLED.show();
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (!modemWasInitialized) {
                    currentCommand.type = LED_GSM_INIT;
                } else if (gprsConnected && !mqttConnected) {
                    currentCommand.type = LED_MQTT_CONNECTING;
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
                FastLED.show();
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (gprsConnected) {
                    currentCommand.type = LED_CONNECTING;
                } else {
                    currentCommand.type = LED_OFFLINE;
                }
                break;
                
            case LED_PUBLISH_EM:
                // Single green blink
                leds[0] = CRGB::Green;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(250));
                leds[0] = CRGB::Black;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(250));
                leds[0] = CRGB::Green;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(250));
                leds[0] = CRGB::Black;
                FastLED.show();
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
                vTaskDelay(pdMS_TO_TICKS(200));
                leds[0] = CRGB::Black;
                FastLED.show();
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (gprsConnected) {
                    currentCommand.type = LED_CONNECTING;
                } else {
                    currentCommand.type = LED_OFFLINE;
                }
                break;

            case LED_DISARMED:
                // Green flash
                leds[0] = CRGB::DeepPink;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::DeepPink;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                FastLED.show();
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
                vTaskDelay(pdMS_TO_TICKS(200));
                leds[0] = CRGB::Black;
                FastLED.show();
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
            {
                //Rainbow animation
                static uint8_t hue = 0;
                leds[0] = CHSV(hue = hue+20, 255, 255);
            }
            break;
            case LED_PING_ACK:
                // Green flash
                leds[0] = CRGB::Green;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Green;
                FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(100));
                leds[0] = CRGB::Black;
                FastLED.show();
                // Return to previous state
                if (deviceOnline) {
                    currentCommand.type = LED_ONLINE;
                } else if (gprsConnected) {
                    currentCommand.type = LED_CONNECTING;
                } else {
                    currentCommand.type = LED_OFFLINE;
                }
                break;

        }
        
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

