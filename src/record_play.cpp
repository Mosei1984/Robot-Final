// record_play.cpp
#include "record_play.h"

RecordPlayManager::RecordPlayManager(const char* filename)
    : _filename(filename) {}

void RecordPlayManager::begin(uint8_t csPin) {
    if (!SD.begin(csPin)) {
        Serial.println("[SD] Karte nicht gefunden!");
    } else {
        Serial.println("[SD] Karte initialisiert.");
    }
}

bool RecordPlayManager::recordPose(const Pose& pose) {
    File file = SD.open(_filename, FILE_WRITE);
    if (!file) {
        Serial.println("[SD] Fehler beim Öffnen der Datei!");
        return false;
    }

    for (int i = 0; i < 6; i++) {
        file.print(pose.jointAngles[i], 4);
        if (i < 5) file.print(",");
    }
    file.println();
    file.close();

    Serial.println("[SD] Pose aufgezeichnet.");
    return true;
}

bool RecordPlayManager::startPlayback() {
    if (_file) _file.close();
    _file = SD.open(_filename);
    if (!_file) {
        Serial.println("[SD] Wiedergabedatei konnte nicht geöffnet werden!");
        return false;
    }
    playing = true;
    Serial.println("[SD] Wiedergabe gestartet.");
    return true;
}

bool RecordPlayManager::getNextPose(Pose& poseOut) {
    if (!_file || !_file.available()) {
        stopPlayback();
        return false;
    }

    String line = _file.readStringUntil('\n');
    int idx = 0;
    char* token = strtok((char*)line.c_str(), ",");

    while (token != nullptr && idx < 6) {
        poseOut.jointAngles[idx++] = atof(token);
        token = strtok(nullptr, ",");
    }

    return (idx == 6);
}

void RecordPlayManager::stopPlayback() {
    if (_file) _file.close();
    playing = false;
    Serial.println("[SD] Wiedergabe beendet.");
}

bool RecordPlayManager::isPlaying() const {
    return playing;
}

void RecordPlayManager::resetFile() {
    SD.remove(_filename);
    File file = SD.open(_filename, FILE_WRITE);
    if (file) file.close();
    Serial.println("[SD] Datei zurückgesetzt.");
}
