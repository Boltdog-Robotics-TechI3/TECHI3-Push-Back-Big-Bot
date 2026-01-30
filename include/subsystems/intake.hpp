#pragma once

#include "main.h"
bool getloadOrLowForwards();

void setloadOrLowForwards(bool stuff);

bool getlowOrHighForwards();

void setlowOrHighForwards(bool stuff);

void toggleLoadOrLow();
void togglelowOrHigh();

// Initializes the intake motor, setting brake mode
void intakeInitialize();

// Polls the controller and determines motor state
void intakePeriodic();

// Sets the speed of the intake motor to the specified parameter
void setIntakeSpeed(int speed);

void intakeStatus();