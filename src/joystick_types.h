#ifndef JOYSTICK_TYPES_H
#define JOYSTICK_TYPES_H

// Define JoystickValues struct in its own header to avoid redefinition
struct JoystickValues {
    float leftX;
    float leftY;
    float rightX;
    float rightY;
    bool leftButton;
    bool rightButton;
};

#endif // JOYSTICK_TYPES_H