#include "Wireless.h"

JsonCommandCallback jsonCommandCallback = nullptr;

#define WIFI_MODE_NONE 0
#define WIFI_MODE_AP_STA 1

// JsonCommandCallback jsonCommandCallback = nullptr;
// JsonDocument jsonCmdReceiveEspnow;
// struct_message espNowMessage;
// struct_message espNowMegsRecv;


// JsonCommandCallback jsonCommandCallback;
JsonDocument jsonCmdReceiveEspnow;


typedef struct struct_message {
    char message[250];
  } struct_message;
struct_message espNowMessage;
struct_message espNowMegsRecv;
esp_now_peer_info_t peerInfo;

int wifiMode = 1;
int maxClients = 1;
bool statusAP = false;
bool statusSTA = false;



bool Wireless::setAP(String ssid, String password, int wifiChannel) {
    if (wifiMode == WIFI_MODE_NONE) {
        Serial.println("WiFi mode is None, skip configuring Access Point");
        return false;
    }
    if (ssid.length() == 0) {
        Serial.println("SSID is empty, skip configuring Access Point");
        return false;
    }
    // WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ssid.c_str(), password.c_str(), wifiChannel, 0, maxClients);
    if (WiFi.softAPIP()) {
        Serial.println("Access Point started");
        Serial.print("IP Address: ");
        Serial.println(WiFi.softAPIP());
        return true;
    } else {
        Serial.println("Failed to start Access Point");
        return false;
    }
}

bool Wireless::setSTA(String ssid, String password) {
    if (wifiMode == WIFI_MODE_NONE) {
        Serial.println("WiFi mode is None, skip configuring Station");
        return false;
    }
    if (ssid.length() == 0) {
        Serial.println("SSID is empty, skip configuring Station");
        return false;
    }
    // WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nFailed to connect to WiFi");
        return false;
    }
}

bool Wireless::setWifiMode(int mode, String ap_ssid, String ap_password, int wifiChannel, String sta_ssid, String sta_password) {
    if(mode == WIFI_MODE_NONE) {
        WiFi.mode(WIFI_OFF);
        wifiMode = WIFI_MODE_NONE;
        Serial.println("WiFi mode set to None");
        return false;
    } else if (mode == WIFI_MODE_AP_STA) {
        WiFi.mode(WIFI_AP_STA);
        wifiMode = WIFI_MODE_AP_STA;
        // setAP(ap_ssid.c_str(), ap_password.c_str(), wifiChannel);
        bool result = WiFi.softAP(ap_ssid.c_str(), ap_password.c_str(), wifiChannel);
        Serial.println(result ? "AP started!" : "AP start failed!");
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        if(setSTA(sta_ssid.c_str(), sta_password.c_str())) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

int Wireless::getRSSI_STA() {
    if (WiFi.status() == WL_CONNECTED) {
        int rssi = WiFi.RSSI();
        return rssi;
    } else {
        return 1;
    }
}

int Wireless::getRSSI_AP() {
    wifi_sta_list_t stationList;
    esp_wifi_ap_get_sta_list(&stationList);
    wifi_sta_info_t station = stationList.sta[0];
    return station.rssi;
}

String Wireless::getAPIP() {
    if (WiFi.softAPIP()) {
        return WiFi.softAPIP().toString();
    } else {
        return "";
    }
}

String Wireless::getSTAIP() {
    if (WiFi.localIP()) {
        return WiFi.localIP().toString();
    } else {
        return "";
    }
}

bool isKnownMac(const uint8_t *mac) {
    for (int i = 0; i < sizeof(knownMacs) / sizeof(knownMacs[0]); i++) {
        if (memcmp(mac, knownMacs[i], 6) == 0) {
            return true;
        }
    }
    return false;
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (!espnowMode) {
        Serial.println("ESP-NOW is off, skip receiving data");
        return;
    }

    if (receivedFromKnownMac) {
        if (!isKnownMac(mac_addr)) {
            // Serial.println("Received data from unknown MAC address, skip processing");
            // Serial0.println("Received data from unknown MAC address, skip processing");
            return;
        }
    }

    memcpy(&espNowMegsRecv, data, sizeof(espNowMegsRecv));

    Serial.print("Bytes received: "); Serial.println(data_len);

    DeserializationError err = deserializeJson(jsonCmdReceiveEspnow, espNowMegsRecv.message);
    if (err == DeserializationError::Ok && jsonCommandCallback != nullptr) {
        jsonCommandCallback(jsonCmdReceiveEspnow);
    }
} 

void Wireless::espnowInit(bool longRange) {
    if (longRange) {
        if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) == ESP_OK) {
            Serial.println("Long Range Mode enabled for STA");
        } else {
            Serial.println("Failed to enable Long Range Mode for STA");
        }

        if (esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR) == ESP_OK) {
            Serial.println("Long Range Mode enabled for AP");
        } else {
            Serial.println("Failed to enable Long Range Mode for AP");
        }
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    // esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *incomingData, int len) {
    //     esp_now_peer_info_t peerInfo;
    //     memcpy(peerInfo.peer_addr, mac, 6);
    //     OnDataRecv(&peerInfo, incomingData, len);
    // });
}

bool Wireless::setEspNowMode(int mode) {
    if (mode == 0) {
        espnowMode = false;
        return true;
    } else if (mode == 1) {
        espnowMode = true;
        return true;
    } else {
        return false;
    }
}

String Wireless::macToString(uint8_t mac[6]) {
    char macStr[18]; // 6 pairs of 2 characters + null terminator
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

uint8_t* Wireless::getMac() {
    static uint8_t mac[6];
    if (esp_efuse_mac_get_default(mac) == ESP_OK) {
        return mac;
    } else {
        Serial.println("Failed to get MAC address");
        return nullptr;
    }
}

void Wireless::macStringToByteArray(const String& macString, uint8_t* byteArray) {
    for (int i = 0; i < 6; i++) {
      byteArray[i] = strtol(macString.substring(i * 3, i * 3 + 2).c_str(), NULL, 16);
    }
    return;
}

bool Wireless::sendEspNow(String macInput, String data) {
    if (!espnowMode) {
        Serial.println("ESP-NOW is off, skip sending data");
        return false;
    }

    if (macInput.length() != 17) {
        Serial.println("invalid MAC address format.");
        return false;
    }
    uint8_t macArray[6];
    macStringToByteArray(macInput, macArray);

    if (esp_now_send(macArray, (uint8_t*)data.c_str(), data.length()) != ESP_OK) {
        Serial.println("Failed to send data");
        return false;
    } else {
        Serial.println("Data sent successfully");
        return true;
    }
}

bool Wireless::sendEspNowJson(uint8_t mac[6], const JsonDocument& jsonCmdInput) {
    if (!espnowMode) {
        Serial.println("ESP-NOW is off, skip sending data");
        return false;
    }

    char outputString[250];
    serializeJson(jsonCmdInput, outputString);

    if (esp_now_send(mac, (uint8_t*)outputString, strlen(outputString)) != ESP_OK) {
        Serial.println("Failed to send data");
        return false;
    } else {
        Serial.println("Data sent successfully");
        return true;
    }
}

void Wireless::setJsonCommandCallback(JsonCommandCallback callback) {
    jsonCommandCallback = callback;
}

void Wireless::addMacToPeerString(String macInput) {
    if (macInput.length() != 17) {
        Serial.println("invalid MAC address format.");
        return;
    }
    uint8_t macArray[6];
    macStringToByteArray(macInput, macArray);

    // esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, macArray, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
    Serial.print("Peer added successfully: ");
    Serial.println(macInput);
}

void Wireless::addMacToPeer(uint8_t mac[6]) {
    // esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
    Serial.print("Peer added successfully: ");
    Serial.println(macToString(mac));
}