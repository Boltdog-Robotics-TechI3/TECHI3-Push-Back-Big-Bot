#include "main.h"

bool loadOrLowForwards = true;
bool lowOrHighForwards = true;

void intakeInitialize() {
    intake.set_brake_mode(MOTOR_BRAKE_COAST);

    intake.set_current_limit(1000);
    midRoller.set_current_limit(1000);
    colorSorter.set_current_limit(1000);
    bottomRollers.set_current_limit(1000);
    loadOrMid.set_current_limit(1000);
    midOrHigh.set_current_limit(2000);
}

void intakePeriodic() {
	if (controller.get_digital(DIGITAL_L1)) { // intake
        setIntakeSpeed(127);
    }
    else if (controller.get_digital(DIGITAL_L2)){
        scoreHigh(-127);
    }
    else if (controller.get_digital(DIGITAL_R1)){
        scoreHigh(127);
    }
    else if (controller.get_digital(DIGITAL_R2)){
        scoreMid(127);
    }
    else {
        setIntakeSpeed(0);
    }
}

void setIntakeSpeed(int speed) {     
    intake.move(speed);
    bottomRollers.move(speed);
    midRoller.move(speed);
    colorSorter.move(speed);
    hopper.move(speed);
    loadOrMid.move(speed);
    midOrHigh.move(0);
}

void scoreMid(int speed) {     
    intake.move(speed);
    bottomRollers.move(speed);
    midRoller.move(speed);
    colorSorter.move(speed);
    hopper.move(speed);
    loadOrMid.move(speed);
    midOrHigh.move(speed);
}

void scoreHigh(int speed) {     
    intake.move(speed);
    bottomRollers.move(speed);
    midRoller.move(speed);
    colorSorter.move(speed);
    hopper.move(speed);
    loadOrMid.move(speed);
    midOrHigh.move(-speed);
}