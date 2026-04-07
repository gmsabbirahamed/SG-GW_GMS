#pragma once
#include "config.h"
#include "led.h"
#include <Bounce2.h>

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

TaskHandle_t buttonTaskHandle = NULL;

QueueHandle_t buttonEventQueue = NULL;

#define BUTTON_TASK_PRIORITY 1

#define BUTTON_TASK_STACK 2048

enum ButtonEventType {
    BUTTON_SINGLE_PRESS,
    BUTTON_DOUBLE_PRESS,
    BUTTON_HOLD_3SEC,
    BUTTON_HOLD_5SEC,
    BUTTON_HOLD_10SEC
};

struct ButtonEvent {
    ButtonEventType type;
    unsigned long press_duration;
};

void buttonTask(void* parameter);

void IRAM_ATTR buttonISR();

void Button_setup() {

    xTaskCreatePinnedToCore(buttonTask, "ButtonTask", BUTTON_TASK_STACK, NULL, BUTTON_TASK_PRIORITY, &buttonTaskHandle, 0);
}

void IRAM_ATTR buttonISR() {
    portENTER_CRITICAL_ISR(&buttonMux);
    buttonInterruptFlag = true;
    portEXIT_CRITICAL_ISR(&buttonMux);
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
                        
                        if (lastPrintedThreshold != 3) {
                            sendLedCommand(LED_BUTTON_HOLD_3SEC);  // Solid cyan
                            SerialMon.println("Hold: 3 seconds reached - LED cyan");
                            lastPrintedThreshold = 3;
                        }
                        break;
                    case 5:
                        
                        if (lastPrintedThreshold != 5) {
                            sendLedCommand(LED_BUTTON_HOLD_5SEC);  // Solid pink
                            SerialMon.println("Hold: 5 seconds reached - LED pink");
                            lastPrintedThreshold = 5;
                        }
                        break;
                    case 10:
                        
                        if (lastPrintedThreshold != 10) {
                            sendLedCommand(LED_BUTTON_HOLD_10SEC);  // Solid purple
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
            }
            else if (holdDuration >= 5000) {
                SerialMon.println("5 SECOND HOLD DETECTED");
                SerialMon.printf("Hold duration: %lu ms\n", holdDuration);
                SerialMon.println("Final LED was: Pink");
            }
            else if (holdDuration >= 3000) {
                SerialMon.println("3 SECOND HOLD DETECTED");
                SerialMon.printf("Hold duration: %lu ms\n", holdDuration);
                SerialMon.println("Final LED was: Cyan");
            }
            else {
                SerialMon.println("SINGLE CLICK DETECTED");
                SerialMon.printf("Hold duration: %lu ms\n", holdDuration);
                SerialMon.println("LED: Yellow blink");
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
