// record_play.h
#ifndef RECORD_PLAY_H
#define RECORD_PLAY_H

#include <Arduino.h>
#include <SD.h>

struct Pose {
    float jointAngles[6];
};

class RecordPlayManager {
public:
    RecordPlayManager(const char* filename = "/motions.csv");
    void begin(uint8_t csPin = BUILTIN_SDCARD);
    bool recordPose(const Pose& pose);
    bool startPlayback();
    bool getNextPose(Pose& poseOut);
    void stopPlayback();
    bool isPlaying() const;
    void resetFile();

private:
    const char* _filename;
    File _file;
    bool playing = false;
};

#endif
