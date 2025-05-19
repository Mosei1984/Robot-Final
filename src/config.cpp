#include "config.h"

SystemState currentState = IDLE;
DisplayMode currentDisplayMode = DISPLAY_SPLASH;

// ======================== Zustand wechseln ========================
void changeState(SystemState newState) {
  currentState = newState;
  Serial.print("State changed to: ");
  Serial.println(currentState);

  switch (newState) {
    case HOMING:
      setDisplayMode(DISPLAY_HOMING);
      break;
    case RECORD:
      setDisplayMode(DISPLAY_RECORD);
      break;
    case PLAY:
      setDisplayMode(DISPLAY_RECORD);
      break;
    case GCODE:
      setDisplayMode(DISPLAY_GCODE);
      break;
    case NORMAL_OPERATION:
      setDisplayMode(DISPLAY_JOINTS);
      break;
    case IDLE:
    default:
      setDisplayMode(DISPLAY_MENU);
      break;
  }
}

void setDisplayMode(DisplayMode mode) {
  currentDisplayMode = mode;
}
