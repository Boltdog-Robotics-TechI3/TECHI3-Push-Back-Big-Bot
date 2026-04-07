#include "main.h"

bool loadOrLowForwards = true;
bool lowOrHighForwards = true;

void intakeInitialize() {
    intake.set_brake_mode(MOTOR_BRAKE_COAST);
    indexer.set_brake_mode(MOTOR_BRAKE_COAST);
    ejector.set_brake_mode(MOTOR_BRAKE_COAST);

    // intake.set_current_limit(1000);
    // indexer.set_current_limit(1000);
    // ejector.set_current_limit(1000);
}

void intakePeriodic() {
	if (controller.get_digital(DIGITAL_L1)) { // intake
        intakeBlocks(127);
    }
    else if (controller.get_digital(DIGITAL_A)) { // outtake
        intakeBlocks(-80);
    }
    else if (controller.get_digital(DIGITAL_R1)) { // mid score
        scoreMid(80);
    }
    else if (controller.get_digital(DIGITAL_R2)){ // high score
        scoreHigh(127);
    }
    else {
        stopIntake();
    }
}

void stopIntake() {
    intake.move(0);
    indexer.move(0);
    ejector.move(0);
}

void intakeBlocks(int speed) {     
    intake.move(speed);
    indexer.move(speed - 50);
    ejector.move(0);
}

void scoreMid(int speed) {     
    intake.move(speed);
    indexer.move(speed);
    ejector.move(-speed);
}

void scoreHigh(int speed) {     
    intake.move(speed);
    indexer.move(speed);
    ejector.move(speed);
}