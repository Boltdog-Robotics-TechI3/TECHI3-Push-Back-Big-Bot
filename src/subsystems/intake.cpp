#include "main.h"
pros::Mutex indexerMutex; 
std::atomic<bool> midScore = false;

void indexerJamHandler(void* pParam);
void colorSortingHandler(void* pParam);

void intakeInitialize() {
    intake.set_brake_mode(MOTOR_BRAKE_COAST);
    indexer.set_brake_mode(MOTOR_BRAKE_COAST);
    ejector.set_brake_mode(MOTOR_BRAKE_COAST);
	pros::Task jamHandler(indexerJamHandler); 
    // pros::Task colorSorting(colorSortingHandler);
}

void indexerJamHandler(void* pParam) {
	int stallTime = -1;
	while (1) {
		if (fabs(indexer.get_actual_velocity()) < 10 && indexer.get_target_velocity() > 0 ||
            fabs(intake.get_actual_velocity()) < 10 && intake.get_target_velocity() > 0) {
			if (stallTime == -1) {
				stallTime = pros::millis();
			} 
            else if (pros::millis() - stallTime >= 150) {
				indexerMutex.take(TIMEOUT_MAX);
				intake.move(-127);
				indexer.move(-127);
				pros::delay(300);
				intake.move(127);
				indexer.move(127);
				stallTime = -1;
				indexerMutex.give();
			}
		} else {
			stallTime = -1;
		}
		pros::delay(20);
	}
}

void colorSortingHandler(void* pParam) {
    colorSensor.set_led_pwm(25);
    while (1) {
        if ((blueAlliance && colorSensor.get_hue() < 30) || (redAlliance && colorSensor.get_hue() > 100)) {
            indexerMutex.take(TIMEOUT_MAX);
            if (midScore) {
                ejector.move(127);
            }
            else {
                ejector.move(-127);
            }
            pros::delay(300);
            if (midScore) {
                ejector.move(-127);
            }
            else {
                ejector.move(127);
            }
            indexerMutex.give();
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

    if (controller.get_digital_new_press(DIGITAL_LEFT)) {
        matchLoadPiston.toggle();
    } 
}

void stopIntake() {
	indexerMutex.take(TIMEOUT_MAX); 
    midScore = false; 
    intake.move(0);
    indexer.move(0);
    ejector.move(0);
	indexerMutex.give();
}

void intakeBlocks(int speed) {  
	indexerMutex.take(TIMEOUT_MAX);   
    intake.move(speed);
    indexer.move(speed - 50);
    ejector.move(0);
	indexerMutex.give();
}

void scoreMid(int speed) {    
	indexerMutex.take(TIMEOUT_MAX);
    midScore = true;
    intake.move(speed);
    indexer.move(speed);
    ejector.move(-speed);
	indexerMutex.give();
}

void scoreHigh(int speed) {   
	indexerMutex.take(TIMEOUT_MAX);  
    intake.move(speed);
    indexer.move(speed);
    ejector.move(speed);
	indexerMutex.give();
}

pros::Task colorSorting = pros::Task([]() {
    while(1) {
        
    }
});
