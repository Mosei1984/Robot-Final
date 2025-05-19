#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <Arduino.h>

struct GCodeCommand {
    char command = '\0';   // z.B. 'G'
    int code = 0;          // z.B. 1 bei G1
    float X = NAN;
    float Y = NAN;
    float Z = NAN;
    float A = NAN;
    float B = NAN;
    float C = NAN;
    float F = NAN;
};

class GCodeParser {
public:
    GCodeParser();
    bool parseLine(const String& line, GCodeCommand& cmdOut);

private:
    float extractValue(const String& line, char key, bool& found);
};

#endif
