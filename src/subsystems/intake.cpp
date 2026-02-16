#include "main.h"

bool loadOrLowForwards = true;
bool lowOrHighForwards = true;

void intakeInitialize() {
    intakes.set_brake_mode(MOTOR_BRAKE_COAST);

    intakes.set_current_limit(1000);
    midRoller.set_current_limit(1000);
    frontRoller.set_current_limit(1000);
    bottomRoller.set_current_limit(1000);
    scoringRoller.set_current_limit(1000);
    topLevel.set_current_limit(2000);
}

void intakePeriodic() {
	if (controller.get_digital(DIGITAL_L1)) { // intake
        intakePiston.extend();
        setIntakeSpeed(127);
    }
    else if (controller.get_digital(DIGITAL_L2)) { // intake
        intakePiston.retract();
        setIntakeSpeed(127);
    }
    else if (controller.get_digital(DIGITAL_X)){
        scoreHigh(-127);
    }
    else if (controller.get_digital(DIGITAL_R1)){
        scoreMid(127);
    }
    else if (controller.get_digital(DIGITAL_R2)){
        scoreHigh(127);
    }
    else {
        setIntakeSpeed(0);
    }
}

void setIntakeSpeed(int speed) {     
    intakes.move(speed);
    bottomRoller.move(speed);
    midRoller.move(speed);
    frontRoller.move(speed);
    topLevel.move(speed);
    scoringRoller.move(0);
}

void scoreMid(int speed) {     
    intakes.move(speed);
    bottomRoller.move(speed);
    midRoller.move(speed);
    frontRoller.move(speed);
    scoringRoller.move(-speed);
    topLevel.move(speed);
}

void scoreHigh(int speed) {     
    intakes.move(speed);
    bottomRoller.move(speed);
    midRoller.move(speed);
    frontRoller.move(speed);
    scoringRoller.move(speed);
    topLevel.move(speed);
}