#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

void initDisplay();
void showStartupScreen();
void showStatus(const String &text);
void showJointValues(const float jointAngles[], int jointCount);
void showMenu(const String* options, int optionCount, int selectedIndex);
void clearDisplay();
void drawBitmap();

#endif
