#pragma once

#include "main.h"

// Initializes the intake motor, setting brake mode
void intakeInitialize();

// Polls the controller and determines motor state
void intakeControlHandler(void* pParams);

// Sets the speed of the intake motor to the specified parameter
void intakeBlocks(int speed);

void stopIntake();

void lowIntaking(int speed);

void lowScore(int speed);

void scoreMid(int speed);

void scoreHigh(int speed);

