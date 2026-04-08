#include "main.h"
pros::Mutex indexorMutex; 
std::atomic<bool> intaking = false;

void indexerJamHandler(void* pParam);

void intakeInitialize() {
    intake.set_brake_mode(MOTOR_BRAKE_COAST);
    indexer.set_brake_mode(MOTOR_BRAKE_COAST);
    ejector.set_brake_mode(MOTOR_BRAKE_COAST);
	pros::Task jamHandler(indexerJamHandler); 
}

void indexerJamHandler(void* pParam) {
	int stallTime = -1;
	while (1) {
		if (intaking) continue;
		if (fabs(indexer.get_actual_velocity()) < 1 && indexer.get_target_velocity() > 0) {
			if (stallTime == -1) {
				stallTime = pros::millis();
			} else if (pros::millis() - stallTime >= 150) {
				indexorMutex.take(TIMEOUT_MAX);
				intake.move(-127);
				indexer.move(-127);
				pros::delay(300);
				intake.move(127);
				indexer.move(127);
				stallTime = -1;
				indexorMutex.give();
			}
		} else {
			stallTime = -1;
		}
		pros::delay(20);
	}
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
	indexorMutex.take(TIMEOUT_MAX);  
    intake.move(0);
    indexer.move(0);
    ejector.move(0);
	indexorMutex.give();
	intaking = false;
}

void intakeBlocks(int speed) {  
	intaking = true;
	indexorMutex.take(TIMEOUT_MAX);   
    intake.move(speed);
    indexer.move(speed - 50);
    ejector.move(0);
	indexorMutex.give();
}

void scoreMid(int speed) {    
	indexorMutex.take(TIMEOUT_MAX);
    intake.move(speed);
    indexer.move(speed);
    ejector.move(-speed);
	indexorMutex.give();
}

void scoreHigh(int speed) {   
	indexorMutex.take(TIMEOUT_MAX);  
    intake.move(speed);
    indexer.move(speed);
    ejector.move(speed);
	indexorMutex.give();
}