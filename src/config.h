#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======================== Systemmodi ========================
enum SystemState {
  IDLE = 0,
  HOMING,
  NORMAL_OPERATION,
  RECORD,
  PLAY,
  GCODE
};

extern SystemState currentState;
void changeState(SystemState newState);

// ======================== EEPROM-Bereiche ========================
#define EEPROM_MAGIC_VALUE 0x42
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_CALIBRATION 10
#define EEPROM_ADDR_HOMING 100

// ======================== Joystick-Pins ========================
#define JOYSTICK_LEFT_X_PIN  41
#define JOYSTICK_LEFT_Y_PIN  40
#define JOYSTICK_RIGHT_X_PIN 38
#define JOYSTICK_RIGHT_Y_PIN 39
#define BUTTON_LEFT          27  // Mode wechseln
#define BUTTON_RIGHT         26  // Bestätigung

// ======================== OLED-Pins (I2C Standard) ========================
#define OLED_ADDR 0x3C

// ======================== Debug-Testpunkte ========================
#ifndef TESTPOINT_0
#define TESTPOINT_0 22
#define TESTPOINT_1 23
#define TESTPOINT_2 24
#define TESTPOINT_3 25
#define TESTPOINT_4 26
#define TESTPOINT_5 27
#define TESTPOINT_SD_FAIL 28
#define TESTPOINT_INIT_DONE 29
#endif

// ======================== Display-Modi ========================
enum DisplayMode {
  DISPLAY_SPLASH,
  DISPLAY_JOINTS,
  DISPLAY_GCODE,
  DISPLAY_RECORD,
  DISPLAY_MENU,
  DISPLAY_HOMING,
  DISPLAY_ERROR
};

extern DisplayMode currentDisplayMode;
void setDisplayMode(DisplayMode mode);

#endif
