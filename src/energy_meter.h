#pragma once
#include "config.h"
#include <HardwareSerial.h>

#include <ModbusMaster.h>

#define USE_SD_CARD false
#define USE_ENERGY_METER
#define USE_SELEC_MFM384 // Uncomment to use Selec MFM384 3-phase meter
// #define USE_DZ81_DZS500 // Uncomment to use DZS500 3-phase meter

// RS485 Serial2 Pins
#define RS485_RX 27
#define RS485_TX 14

#ifdef USE_SELEC_MFM384
    // Modbus register addresses
    #define tNetEnergy_reg_addr 0x3A
    #define tImpEnergy_reg_addr 0x60
    #define activePower_reg_addr 0x2A
    #define pAvolt_reg_addr 0x00
    #define pBvolt_reg_addr 0x02
    #define pCvolt_reg_addr 0x04
    #define lABvolt_reg_addr 0x08
    #define lBCvolt_reg_addr 0x0A
    #define lCAvolt_reg_addr 0x0C
    #define pAcurrent_reg_addr 0x10
    #define pBcurrent_reg_addr 0x12
    #define pCcurrent_reg_addr 0x14
    #define frequency_reg_addr 0x38
    #define powerfactor_reg_addr 0x36

#elif defined(USE_DZ81_DZS500)
    // Modbus register addresses for 3-phase meter
    #define taeHigh_reg_addr     0x30
    #define taeLow_reg_addr      0x31
    #define activePower_reg_addr 0x1A
    #define pAvolt_reg_addr      0x14
    #define pBvolt_reg_addr      0x15
    #define pCvolt_reg_addr      0x16
    #define lABvolt_reg_addr     0x17
    #define lBCvolt_reg_addr     0x18
    #define lCAvolt_reg_addr     0x19
    #define pAcurrent_reg_addr   0x10
    #define pBcurrent_reg_addr   0x11
    #define pCcurrent_reg_addr   0x12
    #define frequency_reg_addr   0x1E
    #define powerfactor_reg_addr 0x1D
#endif

#ifdef USE_SELEC_MFM384
    // Modbus data variables
    float tNetEnergy, tImpEnergy, activePower;
    float pAvolt, pBvolt, pCvolt;
    float lABvolt, lBCvolt, lCAvolt;
    float pAcurrent, pBcurrent, pCcurrent;
    float frequency, powerFactor;
#elif defined(USE_DZ81_DZS500)
// Data variables
    int taeHigh, taeLow, activePower;
    int pAvolt, pBvolt, pCvolt;
    int lABvolt, lBCvolt, lCAvolt;
    int pAcurrent, pBcurrent, pCcurrent;
    int frequency, powerFactor;
#endif

char em_data[128];
ModbusMaster node;

#ifdef USE_SELEC_MFM384
    float readModbusData(uint16_t regAddress, uint8_t maxRetries) {
    
    delay(150);
    while (maxRetries > 0) {
        uint8_t result = node.readInputRegisters(regAddress, 2);
        if (result == node.ku8MBSuccess) {
            uint16_t lowWord = node.getResponseBuffer(0);  // LSB stored in lower register
            uint16_t highWord = node.getResponseBuffer(1); // MSB stored in higher register

            union {
                uint32_t intVal;
                float floatVal;
            } converter;

            converter.intVal = ((uint32_t)highWord << 16) | lowWord;
            Serial.printf("Modbus Read Success: Reg 0x%04X, Value: %.2f\n", regAddress, converter.floatVal);
            return converter.floatVal; // Return value if read is successful
        } else {
            maxRetries--;
            Serial.println("Modbus Read Error, Retrying...");
            delay(100); // Optionally add a delay between retries
        }
    }
    
    // If all retries failed, return NaN to indicate an error
    Serial.println("Modbus Read Failed after retries");
    return NAN;
    }

#elif defined(USE_DZ81_DZS500)
  // Function to read Modbus data from the RS485 slave
    int readModbusData(uint16_t reg_address, uint8_t max_retries) {
    vTaskDelay(pdMS_TO_TICKS(30));
    int value = -1;
    while (max_retries-- > 0) {
        uint8_t result = node.readHoldingRegisters(reg_address, 1);
        if (result == node.ku8MBSuccess) {
            value = node.getResponseBuffer(0);
            Serial.print("Modbus Read 0x");
            Serial.print(reg_address, HEX);
            Serial.print(": ");
            Serial.println(value);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    return value;
    }
#endif

#ifdef USE_SELEC_MFM384
  // Getting Modbus Data
    void getModbusData() {
        // Read Modbus data with specific retry counts for each field
        tNetEnergy = readModbusData(tNetEnergy_reg_addr, 2);       // Retry up to 3 times
        tImpEnergy = readModbusData(tImpEnergy_reg_addr, 3);         // Retry up to 3 times
        activePower = readModbusData(activePower_reg_addr, 2); // Retry up to 2 times
        pAvolt = readModbusData(pAvolt_reg_addr, 1);         // Retry up to 1 times
        pBvolt = readModbusData(pBvolt_reg_addr, 1);         // Retry up to 1 times
        pCvolt = readModbusData(pCvolt_reg_addr, 1);         // Retry up to 2 times
        lABvolt = readModbusData(lABvolt_reg_addr, 1);       // Retry up to 1 time
        lBCvolt = readModbusData(lBCvolt_reg_addr, 1);       // Retry up to 1 time
        lCAvolt = readModbusData(lCAvolt_reg_addr, 1);       // Retry up to 1 time
        pAcurrent = readModbusData(pAcurrent_reg_addr, 1);   // Retry up to 2 times
        pBcurrent = readModbusData(pBcurrent_reg_addr, 1);   // Retry up to 2 times
        pCcurrent = readModbusData(pCcurrent_reg_addr, 1);   // Retry up to 2 times
        frequency = readModbusData(frequency_reg_addr, 2);   // Retry up to 1 time
        powerFactor = readModbusData(powerfactor_reg_addr, 2); // Retry up to 1 time
    }
#elif defined(USE_DZ81_DZS500)
    // Function to initialize Modbus communication
    void getModbusData(){
        taeHigh     = readModbusData(taeHigh_reg_addr, 3);
        taeLow      = readModbusData(taeLow_reg_addr, 3);
        activePower = readModbusData(activePower_reg_addr, 2);
        pAvolt      = readModbusData(pAvolt_reg_addr, 1);
        pBvolt      = readModbusData(pBvolt_reg_addr, 1);
        pCvolt      = readModbusData(pCvolt_reg_addr, 2);
        lABvolt     = readModbusData(lABvolt_reg_addr, 1);
        lBCvolt     = readModbusData(lBCvolt_reg_addr, 1);
        lCAvolt     = readModbusData(lCAvolt_reg_addr, 1);
        pAcurrent   = readModbusData(pAcurrent_reg_addr, 1);
        pBcurrent   = readModbusData(pBcurrent_reg_addr, 1);
        pCcurrent   = readModbusData(pCcurrent_reg_addr, 1);
        frequency   = readModbusData(frequency_reg_addr, 1);
        powerFactor = readModbusData(powerfactor_reg_addr, 1);

        // Serial.println("--------- Modbus Data ---------");
        // Serial.printf("taeHigh: %d\n", taeHigh);
        // Serial.printf("taeLow: %d\n", taeLow);
        // Serial.printf("Active Power: %d\n", activePower);
        // Serial.printf("Phase A Voltage: %d\n", pAvolt);
        // Serial.printf("Phase B Voltage: %d\n", pBvolt);
        // Serial.printf("Phase C Voltage: %d\n", pCvolt);
        // Serial.printf("Line AB Voltage: %d\n", lABvolt);
        // Serial.printf("Line BC Voltage: %d\n", lBCvolt);
        // Serial.printf("Line CA Voltage: %d\n", lCAvolt);
        // Serial.printf("Phase A Current: %d\n", pAcurrent);
        // Serial.printf("Phase B Current: %d\n", pBcurrent);
        // Serial.printf("Phase C Current: %d\n", pCcurrent);
        // Serial.printf("Frequency: %d Hz\n", frequency);
        // Serial.printf("Power Factor: %d\n", powerFactor);
        // Serial.println("--------------------------------");
    }
#endif

#ifdef USE_SELEC_MFM384
    // Parsing Modbus Data
    void ParsingModbusData() {
    // Format the data into the buffer with DEVICE_ID at the beginning
    snprintf(em_data, sizeof(em_data), 
            "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            DEVICE_ID.c_str(),  // DEVICE_ID
            tNetEnergy,
            tImpEnergy,
            activePower,
            pAvolt,
            pBvolt,
            pCvolt,
            lABvolt,
            lBCvolt,
            lCAvolt,
            pAcurrent,
            pBcurrent,
            pCcurrent,
            frequency,
            powerFactor);
}
#elif defined(USE_DZ81_DZS500)
    // Function to parse Modbus data and prepare MQTT message
    void ParsingModbusData() {
        // Combine 32-bit energy register from taeHigh and taeLow
        uint32_t totalEnergyRaw = ((uint32_t)taeHigh << 16) | (uint32_t)taeLow;

        // Scale energy — assuming it's in 0.1 kWh units
        float totaltNetEnergy = totalEnergyRaw * 0.1;
        float tImpEnergy = totalEnergyRaw * 0.1;

        // Apply proper scaling
        float ap = activePower * 0.1;
        float va = pAvolt * 0.1;
        float vb = pBvolt * 0.1;
        float vc = pCvolt * 0.1;
        float vab = lABvolt * 0.1;
        float vbc = lBCvolt * 0.1;
        float vca = lCAvolt * 0.1;
        float ia = pAcurrent * 0.1;
        float ib = pBcurrent * 0.1;
        float ic = pCcurrent * 0.1;
        float freq = frequency * 0.01;
        float pf = powerFactor * 0.001;

        // Optional debug prints to verify scaling
        // Serial.println("---- Scaled Values ----");
        // Serial.printf("Energy: %.2f kWh, Power: %.2f W\n", totaltNetEnergy, ap);
        // Serial.printf("Voltages: VA=%.2f, VB=%.2f, VC=%.2f, VAB=%.2f, VBC=%.2f, VCA=%.2f\n", va, vb, vc, vab, vbc, vca);
        // Serial.printf("Currents: IA=%.2f, IB=%.2f, IC=%.2f\n", ia, ib, ic);
        // Serial.printf("Frequency: %.2f Hz, PF: %.3f\n", freq, pf);

        // Format the final MQTT message string
        snprintf(em_data, sizeof(em_data),
        "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        DEVICE_ID.c_str(),
        totaltNetEnergy, tImpEnergy, ap,
        va, vb, vc,
        vab, vbc, vca,
        ia, ib, ic,
        freq, pf);
    }
#endif

void energy_meter_setup(){
    Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
    node.begin(1, Serial2);
}