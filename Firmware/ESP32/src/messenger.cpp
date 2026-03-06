/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: Contains code for sending and receiving data over Bluetooth connection
 * 
 * Note: This code was generated in part by AI tools (chatGPT)
 ************************************************************/

 #include <Arduino.h>

#include "messenger.h"
#include "commands.h"   
#include "state.h"      

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


static const char* BLE_DEVICE_NAME   = "ParkingBot";
static const char* BLE_SERVICE_UUID  = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* BLE_CHAR_UUID_RX  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // phone -> ESP32
static const char* BLE_CHAR_UUID_TX  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // ESP32 -> phone

static BLEServer*         pServer           = nullptr;
static BLECharacteristic* pTxCharacteristic = nullptr;
static bool deviceConnected                 = false;

// ============= Server connect/disconnect callbacks =============
class BotServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override {
        deviceConnected = true;
        Serial.println("[BLE] Central connected");
    }
    void onDisconnect(BLEServer* server) override {
        deviceConnected = false;
        Serial.println("[BLE] Central disconnected, restarting advertising");
        BLEDevice::startAdvertising();
    }
};

// ============= RX characteristic callbacks (phone -> ESP32) =============
class BotRXCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.empty()) return;

        // Convert to Arduino String and trim
        String cmd = String(rxValue.c_str());
        cmd.trim();

        Serial.print("[BLE RX] ");
        Serial.println(cmd);

        if (cmd.length() > 0) {
            handleUserCommand(cmd);
        }
    }
};

// ============= Public functions =============

void initBluetooth() {
    // Initialize BLE stack
    BLEDevice::init(BLE_DEVICE_NAME);

    // Create BLE server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BotServerCallbacks());

    // Create UART-like service
    BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

    // TX characteristic (notify) - ESP32 -> phone
    pTxCharacteristic = pService->createCharacteristic(
        BLE_CHAR_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());  // required for iOS to enable notifications

    // RX characteristic (write) - phone -> ESP32
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        BLE_CHAR_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new BotRXCallbacks());

    // Start service
    pService->start();

    // Start advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // helps with iOS connections
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] UART service started, waiting for iPhone to connect...");
}

void wifiUpdate() {
}

void sendLog(const String &msg) {
    // Always print to Serial
    Serial.println(msg);

    // Also send over BLE if a central is connected
    if (deviceConnected && pTxCharacteristic != nullptr) {
        std::string s(msg.c_str());
        pTxCharacteristic->setValue(s);
        pTxCharacteristic->notify();

        // Small delay to give BLE stack breathing room
        delay(5);
    }
}
