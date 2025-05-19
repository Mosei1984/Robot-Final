#include "TimerLoop.h"

namespace {
    volatile bool doControl = false;
    unsigned long lastJoystickMs = 0, lastDisplayMs = 0;
    void (*controlCallback)() = nullptr;

    void controlISR() {
        doControl = true;
    }
}

namespace TimerLoop {
    void begin(void (*controlFunc)()) {
        controlCallback = controlFunc;
        Timer1.initialize(1000); // 1ms = 1kHz
        Timer1.attachInterrupt(controlISR);
    }

    void loop(void (*joystickFunc)(), void (*displayFunc)()) {
        unsigned long now = millis();

        // 1kHz control tasks
        if (doControl) {
            doControl = false;
            if (controlCallback) controlCallback();
        }

        // 100Hz joystick sampling
        if (now - lastJoystickMs >= 10) {
            lastJoystickMs = now;
            if (joystickFunc) joystickFunc();
        }

        // 50Hz display refresh
        if (now - lastDisplayMs >= 20) {
            lastDisplayMs = now;
            if (displayFunc) displayFunc();
        }
    }
}