#include "gcode_parser.h"

GCodeParser::GCodeParser() {}

bool GCodeParser::parseLine(const String& line, GCodeCommand& cmdOut) {
    String cleanLine = line;
    cleanLine.trim();

    if (cleanLine.length() == 0 || cleanLine.charAt(0) == ';') return false;

    cmdOut = GCodeCommand();

    for (size_t i = 0; i <cleanLine.length(); i++) {
        char c = toupper(cleanLine[i]);

        if (c == 'G' || c == 'M') {
            unsigned j = i + 1;
            String number = "";
            while (j < cleanLine.length() && isDigit(cleanLine[j])) {
                number += cleanLine[j++];
            }

            cmdOut.command = c;
            cmdOut.code = number.toInt();
        }

        bool found = false;

        cmdOut.X = isnan(cmdOut.X) ? extractValue(cleanLine, 'X', found) : cmdOut.X;
        cmdOut.Y = isnan(cmdOut.Y) ? extractValue(cleanLine, 'Y', found) : cmdOut.Y;
        cmdOut.Z = isnan(cmdOut.Z) ? extractValue(cleanLine, 'Z', found) : cmdOut.Z;
        cmdOut.A = isnan(cmdOut.A) ? extractValue(cleanLine, 'A', found) : cmdOut.A;
        cmdOut.B = isnan(cmdOut.B) ? extractValue(cleanLine, 'B', found) : cmdOut.B;
        cmdOut.C = isnan(cmdOut.C) ? extractValue(cleanLine, 'C', found) : cmdOut.C;
        cmdOut.F = isnan(cmdOut.F) ? extractValue(cleanLine, 'F', found) : cmdOut.F;
    }

    return true;
}

float GCodeParser::extractValue(const String& line, char key, bool& found) {
    key = toupper(key);
    int pos = line.indexOf(key);
    if (pos == -1) {
        found = false;
        return NAN;
    }

    int start = pos + 1;
    unsigned end = start;
    while (end < line.length() && (isDigit(line[end]) || line[end] == '.' || line[end] == '-')) {
        end++;
    }

    String valueStr = line.substring(start, end);
    found = true;
    return valueStr.toFloat();
}
