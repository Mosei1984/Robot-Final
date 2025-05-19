#include <Arduino.h>
#include "RobotKinematics.h"

// Data structure to store robot movements
struct MovementData {
    unsigned long timestamp;
    JointAngles jointPositions;
};

namespace RecordPlay {
    // Initialize record/playback system
    void init();
    
    // Start/stop recording
    bool startRecording();
    void stopRecording();
    
    // Playback functions
    bool startPlayback();
    void stopPlayback();
    
    // Get state
    bool isRecording();
    bool isPlaying();
    
    // Save/load recordings
    bool saveRecording(const char* filename);
    bool loadRecording(const char* filename);
    
    // Update function called in main loop
    void update();
}
