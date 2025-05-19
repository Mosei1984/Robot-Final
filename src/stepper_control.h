#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "axis_config.h"

extern AxisConfig axisConfigs[NUM_AXES];
extern AccelStepper* steppers[NUM_AXES];

void setupAxes();
void enableAllSteppers();
void disableAllSteppers();
bool isEndstopTriggered(int axis);
void resetAllStepperPositions();

#endif
