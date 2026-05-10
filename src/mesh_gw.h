#pragma once

#include <WiFi.h>
#include <esp_now.h>
#include <deque>
#include <algorithm>
#include "config.h"
#include "led.h"

const char* Local_ID = "gw0"; // Gateway ID
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct Message {
    String sender_id;
    String receiver_id;
    String command;
    String type;
    String msg_id;
};

std::deque<String> recentMsgKeys;
const size_t maxRecentIDs = 20;

// Function prototypes
void mesh_gw_setup();
bool isDuplicate(const String& msg_id);
String generateMessageID();
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);


void mesh_gw_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW Init Failed");
        return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    esp_now_register_recv_cb(onReceive);
}

// Function to check for duplicate ACKs
bool isDuplicate(const String& type, const String& msg_id) {
    String key = type + ":" + msg_id;
    if (std::find(recentMsgKeys.begin(), recentMsgKeys.end(), key) != recentMsgKeys.end()) {
        return true;
    }
    recentMsgKeys.push_back(key);
    if (recentMsgKeys.size() > maxRecentIDs) {
        recentMsgKeys.pop_front();
    }
    return false;
}

// Function to generate a unique message ID
String generateMessageID() {
    uint16_t randNum = esp_random() & 0xFFFF;
    // Serial.print("Raw 16-bit randNum: ");
    // Serial.println(randNum);
    char id[5];
    sprintf(id, "%04X", randNum);
    return String(id);
    }

// Callback function for receiving ESP-NOW messages
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {

    String msg((char*)incomingData, len);
    Serial.println("\n📥 Received: " + msg);

    int commaCount = std::count(msg.begin(), msg.end(), ',');
    if (commaCount != 4) {
        Serial.println("❌ Invalid message format. Skipped.");
        return;
    }

    int idx1 = msg.indexOf(',');
    int idx2 = msg.indexOf(',', idx1 + 1);
    int idx3 = msg.indexOf(',', idx2 + 1);
    int idx4 = msg.indexOf(',', idx3 + 1);

    String sender_id   = msg.substring(0, idx1);
    String receiver_id = msg.substring(idx1 + 1, idx2);
    String command     = msg.substring(idx2 + 1, idx3);
    String type        = msg.substring(idx3 + 1, idx4);
    String msg_id      = msg.substring(idx4 + 1);
    
    // 🔁 Deduplication for ALL types
    if (isDuplicate(type, msg_id)) {
        Serial.println("⚠️ Duplicate " + type + " ignored (id=" + msg_id + ")");
        return;
    }

    // Only process known types
    if (type != "ack" && type != "hb" && type != "tmp") {
        Serial.println("⏭ Ignored unknown type: " + type);
        return;
    }

    Serial.printf("✅ %s Received: sender=%s → receiver=%s | cmd=%s | id=%s\n",
                    type.c_str(), sender_id.c_str(), receiver_id.c_str(),
                    command.c_str(), msg_id.c_str());

    // Prepare MQTT message
    MQTTMessage mqttMsg;
    if (type == "ack") {
        snprintf(mqttMsg.topic, sizeof(mqttMsg.topic), MQTT_AC_ACK);
        snprintf(mqttMsg.payload, sizeof(mqttMsg.payload), "%s,%s,%s", String(DEVICE_ID).c_str(), sender_id.c_str(), command.c_str());
        SerialMon.printf("📥 ACK: %s", mqttMsg.payload);
        
        sendLedCommand(LED_PING_ACK);
    } 
    
    else if (type == "hb") {
        snprintf(mqttMsg.topic, sizeof(mqttMsg.topic), MQTT_AC_HB);
        snprintf(mqttMsg.payload, sizeof(mqttMsg.payload), "%s,%s,%s", String(DEVICE_ID).c_str(), sender_id.c_str(), command.c_str());
        SerialMon.printf("📥 ACK: %s", mqttMsg.payload);
    } 
    else if (type == "tmp") {
        snprintf(mqttMsg.topic, sizeof(mqttMsg.topic), MQTT_AC_TMP);
        snprintf(mqttMsg.payload, sizeof(mqttMsg.payload), "%s,%s,%s", String(DEVICE_ID).c_str(), sender_id.c_str(), command.c_str());
        SerialMon.printf("📥 ACK: %s", mqttMsg.payload);
    }

    xQueueSend(mqttPublishQueue, &mqttMsg, pdMS_TO_TICKS(100));
    
    // Re-broadcast if needed
    // rebroadcastIfNeeded(msg_id, type, msg);
}
