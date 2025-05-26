#ifndef WIRELESS_H
#define WIRELESS_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

typedef void (*JsonCommandCallback)(const JsonDocument& jsonCmdInput);
static uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t knownMacs[][6] = {
    {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF},
    {0x24, 0x6F, 0x28, 0x12, 0x34, 0x56}
};
static bool receivedFromKnownMac = false;
// esp-now, on -> true, off -> false
static bool espnowMode = true;

class Wireless{
    public:
        bool setAP(String ssid, String password, int wifiChannel);
        bool setSTA(String ssid, String password);
        bool setWifiMode(int mode, String ap_ssid, String ap_password, int wifiChannel, String sta_ssid, String sta_password);
        int getRSSI_AP();
        int getRSSI_STA();
        String getAPIP();
        String getSTAIP();

        void espnowInit(bool longRange);
        bool setEspNowMode(int mode);
        void macStringToByteArray(const String& macString, uint8_t* byteArray);
        String macToString(uint8_t mac[6]);
        uint8_t* getMac();
        bool sendEspNow(String macInput, String data);
        bool sendEspNowJson(uint8_t mac[6], const JsonDocument& jsonCmdInput);
        void setJsonCommandCallback(JsonCommandCallback callback);
        void addMacToPeerString(String macInput);
        void addMacToPeer(uint8_t mac[6]);
};

#endif