#include "gcode_parser.h"
#include <Arduino.h>

// Struktur zur Speicherung eines GCode-Kommandos und aller Achsenwerte
struct GCodeCommand {
    char command = 0;   // 'G' oder 'M'
    int code = -1;      // G- oder M-Nummer
    float X = NAN;
    float Y = NAN;
    float Z = NAN;
    float A = NAN;      // Yaw
    float B = NAN;      // Pitch
    float C = NAN;      // Roll
    float F = NAN;      // Feedrate/Geschwindigkeit

    GCodeCommand() {}
};

class GCodeParser {
public:
    GCodeParser();

    // Zeile parsen, Ergebnis in cmdOut schreiben
    bool parseLine(const String& line, GCodeCommand& cmdOut);

    // Hilfsfunktion zum Extrahieren eines Wertes
    float extractValue(const String& line, char key, bool& found);
};
