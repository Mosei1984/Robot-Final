#pragma once

#include <Arduino.h>
#include <SD.h>
#include "RobotKinematics.h"

#define MAX_POSITIONS 100
#define RECORD_INTERVAL 200  // ms between position recordings

namespace RecordPlaySystem {
    struct MovementData {
        uint32_t timestamp;   // Time in ms when position was recorded
        JointAngles joints;   // Joint angles
    };
    
    // Initialize record/play system
    void init();
    
    // Record and playback control
    void startRecording();
    void stopRecording();
    bool isRecording();
    
    void startPlayback();
    void stopPlayback();
    bool isPlaying();
    
    // Update - call in main loop
    void update();
    
    // File management
    bool saveRecording(const char* filename);
    bool loadRecording(const char* filename);
    void listRecordings();
    
    // Recording configuration
    void setRecordInterval(uint32_t interval);
    uint32_t getRecordInterval();
}
