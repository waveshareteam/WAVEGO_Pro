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
            flash();
        }
        if (LittleFS.begin()) {
            Serial.println("LittleFS.begin() success");
        } else {
            Serial.println("LittleFS.begin() failed");
            flash();
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
        File file = LittleFS.open("/" + missionName + ".mission", "w");
        if (file) {
            file.print(content + "\n");
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
        return;
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
		Serial.print("[StepNum: ");Serial.print(_lineNum);Serial.print(" ] - ");
		Serial.println(line);
    }
    file.close();
}

void FilesCtrl::appendStep(String missionName, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return;
    }
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

void FilesCtrl::insertStep(String missionName, int stepNum, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return;
    }
    DeserializationError err = deserializeJson(cmdJson, jsonCmd);
    if (err == DeserializationError::Ok) {
        Serial.println("json parsing succeed.");
        File file = LittleFS.open("/" + missionName + ".mission", "r");
        if(!file){
        Serial.println("Error opening file for inserting.");
        return;
        }
        String _content = "";
        _content += file.readStringUntil('\n') + "\n";
        for (int i = 0; i < stepNum - 1; i++) {
            _content += file.readStringUntil('\n') + "\n";
        }
        _content += jsonCmd + "\n";
        while (file.available()) {
            _content += file.readStringUntil('\n') + "\n";
        }
        file.close();
        file = LittleFS.open("/" + missionName + ".mission", "w");
        if(!file){
        Serial.println("Error opening file for inserting.");
        return;
        }
        file.print(_content);
        file.close();
        Serial.println("=== Edited File ===");
        FilesCtrl::missionContent(missionName);
        cmdJson.clear();
    } else {
        cmdJson.clear();
        Serial.println("[deserializeJson err]");
    }
}

void FilesCtrl::replaceStep(String missionName, int stepNum, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return;
    }
    if (stepNum < 1) {
        Serial.println("stepNum should be greater than 0.");
        return;
    }
    DeserializationError err = deserializeJson(cmdJson, jsonCmd);
    if (err == DeserializationError::Ok) {
        Serial.println("json parsing succeed.");
        File file = LittleFS.open("/" + missionName + ".mission", "r");
        if(!file){
        Serial.println("Error opening file for replacing.");
        return;
        }
        String _content = "";
        _content += file.readStringUntil('\n') + "\n";
        for (int i = 0; i < stepNum - 1; i++) {
            _content += file.readStringUntil('\n') + "\n";
        }
        _content += jsonCmd + "\n";
        file.readStringUntil('\n');
        while (file.available()) {
            _content += file.readStringUntil('\n') + "\n";
        }
        file.close();
        file = LittleFS.open("/" + missionName + ".mission", "w");
        if(!file){
        Serial.println("Error opening file for replacing.");
        return;
        }
        file.print(_content);
        file.close();
        Serial.println("=== Edited File ===");
        FilesCtrl::missionContent(missionName);
        cmdJson.clear();
    } else {
        cmdJson.clear();
        Serial.println("[deserializeJson err]");
    }
}

void FilesCtrl::deleteStep(String missionName, int stepNum) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return;
    }
    if (stepNum < 1) {
        Serial.println("stepNum should be greater than 0.");
        return;
    }
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if(!file){
        Serial.println("Error opening file for deleting.");
        return;
    }
    String _content = "";
    _content += file.readStringUntil('\n') + "\n";
    for (int i = 0; i < stepNum - 1; i++) {
        _content += file.readStringUntil('\n') + "\n";
    }
    file.readStringUntil('\n');
    while (file.available()) {
        _content += file.readStringUntil('\n') + "\n";
    }
    file.close();
    file = LittleFS.open("/" + missionName + ".mission", "w");
    if(!file){
        Serial.println("Error opening file for deleting.");
        return;
    }
    file.print(_content);
    file.close();
    Serial.println("=== Edited File ===");
    FilesCtrl::missionContent(missionName);
}

String FilesCtrl::readStep(String missionName, int stepNum) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return "";
    }
    if (stepNum < 1) {
        Serial.println("stepNum should be greater than 0.");
        return "";
    }
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if(!file){
        Serial.println("Error opening file for reading.");
        return "";
    }
    String _content = "";
    file.readStringUntil('\n');
    for (int i = 0; i < stepNum - 1; i++) {
        file.readStringUntil('\n');
    }
    _content += file.readStringUntil('\n');
    file.close();
    return _content;
}

void FilesCtrl::deleteMission(String missionName) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return;
    }
    LittleFS.remove("/" + missionName + ".mission");
    Serial.println("file deleted.");
}

bool FilesCtrl::checkMission(String missionName) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        return false;
    }
    return true;
}

bool FilesCtrl::checkStepByType(String missionName, int cmdType) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        return false;
    }
    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if(!file){
        Serial.println("Error opening file for reading.");
        return false;
    }
    file.readStringUntil('\n');
    while (file.available()) {
        String _content = file.readStringUntil('\n');
        DeserializationError err = deserializeJson(cmdJson, _content);
        if (err == DeserializationError::Ok) {
            if (cmdJson["T"].as<int>() == cmdType) {
                cmdJson.clear();
                return true;
            }
        }
        cmdJson.clear();
    }
    return false;
}

bool FilesCtrl::checkReplaceStep(String missionName, String jsonCmd) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return false;
    }

    cmdJson.clear();
    DeserializationError err = deserializeJson(cmdJson, jsonCmd);
    if (err != DeserializationError::Ok) {
        Serial.println("[deserializeJson err]");
        return false;
    }

    if (!cmdJson["T"].is<int>()) {
        Serial.println("Invalid jsonCmd: missing 'T' field.");
        return false;
    }
    int targetT = cmdJson["T"].as<int>();

    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if (!file) {
        Serial.println("Error opening file for reading.");
        return false;
    }

    String _content = "";
    String line;
    bool replaced = false;
    _content += file.readStringUntil('\n') + "\n";

    // Read and process each line
    while (file.available()) {
        line = file.readStringUntil('\n');
        cmdJson.clear();
        DeserializationError lineErr = deserializeJson(cmdJson, line);

        if (lineErr == DeserializationError::Ok && cmdJson["T"].as<int>() == targetT) {
            replaced = true; // Mark as replaced for each matching line
        } else {
            _content += line + "\n"; // Keep the line if not matching
            Serial.print("read: ");
            Serial.println(cmdJson["T"].as<int>());
            Serial.print("target: ");
            Serial.println(targetT);
        }
    }
    file.close();

    // Write the updated content back to the file
    file = LittleFS.open("/" + missionName + ".mission", "w");
    if (!file) {
        Serial.println("Error opening file for writing.");
        return false;
    }
    file.print(_content);
    file.close();

    // Append the new jsonCmd if replacement occurred
    if (replaced) {
        appendStep(missionName, jsonCmd);
        Serial.println("Step replaced successfully.");
        return true;
    } else {
        appendStep(missionName, jsonCmd);
        Serial.println("No matching step found to replace. Appended instead.");
        return true;
    }
}

String FilesCtrl::findCmdByType(String missionName, int cmdType) {
    if (!LittleFS.exists("/" + missionName + ".mission")) {
        Serial.println("file not found.");
        return "";
    }

    File file = LittleFS.open("/" + missionName + ".mission", "r");
    if (!file) {
        Serial.println("Error opening file for reading.");
        return "";
    }

    String line;
    String targetCmd = "";
    while (file.available()) {
        line = file.readStringUntil('\n');
        DeserializationError lineErr = deserializeJson(cmdJson, line);
        if (lineErr == DeserializationError::Ok && cmdJson["T"] == cmdType) {
            targetCmd = line;
            break;
        }
    }
    file.close();

    return targetCmd;
}