#include "FilesCtrl.h"
#include <nvs_flash.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

JsonDocument cmdJson;

void FilesCtrl::init() {
    if (!flashStatus) {
        if (nvs_flash_init() == ESP_OK) {
            Serial.println("nvs_flash_init() success");
        } else {
            Serial.println("nvs_flash_init() failed");
        }
        if (LittleFS.begin()) {
            Serial.println("LittleFS.begin() success");
        } else {
            Serial.println("LittleFS.begin() failed");
        }
        flashStatus = true;
    }
}

void FilesCtrl::flash() {
    if (flashStatus) {
        LittleFS.format();
        Serial.println("=== LittleFS formatted ===");
    }
}

void FilesCtrl::scan() {
    if (flashStatus) {
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        Serial.println("=== Scaning Files ===");
        while (file) {
            Serial.print("file name: ");
            Serial.print(file.name());
            Serial.print(" file size: ");
            Serial.println(file.size());
            file = root.openNextFile();
        }
    }
}

void FilesCtrl::createMission(String missionName, String content) {
    if (flashStatus) {
        if (LittleFS.exists("/" + missionName + ".mission")) {
            Serial.println("file already exists");
            return;
        }
        File file = LittleFS.open("/" + missionName, "w");
        if (file) {
            file.print(content);
            file.close();
            Serial.println("file created");
        } else {
            Serial.println("file creation failed");
        }
    }
}

void FilesCtrl::missionContent(String missionName) {
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if (!file) {
        Serial.println("file not found.");
    }
    Serial.println("=== File Content ===");
    Serial.print("File Name: ");
    Serial.println(missionName);
    Serial.print("Intro: ");
    Serial.println(file.readStringUntil('\n'));
    int _lineNum = 0;
    while (file.available()) {
		_lineNum++;
		String line = file.readStringUntil('\n');
		Serial.print("[lineNum: ");Serial.print(_lineNum);Serial.print(" ] - ");
		Serial.println(line);
    }
    file.close();
}

void FilesCtrl::appendStep(String missionName, String jsonCmd) {
	DeserializationError err = deserializeJson(cmdJson, jsonCmd);
	if (err == DeserializationError::Ok) {
		Serial.println("json parsing succeed.");
        File file = LittleFS.open("/" + missionName + ".mission", "a");
        if(!file){
        Serial.println("Error opening file for appending.");
        return;
        }
        file.println(jsonCmd);
        file.close();
        Serial.println("=== Edited File ===");
        FilesCtrl::missionContent(missionName);
		cmdJson.clear();
	} else {
		cmdJson.clear();
		Serial.println("[deserializeJson err]");
	}
}