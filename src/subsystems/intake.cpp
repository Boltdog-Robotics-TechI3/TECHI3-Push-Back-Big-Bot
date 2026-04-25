#include "main.h"
pros::Mutex indexerMutex; 
std::atomic<bool> midScore = false;
std::atomic<bool> outtaking = false;
std::atomic<bool> intaking = false;

void indexerJamHandler(void* pParam);
void colorSortingHandler(void* pParam);

void intakeInitialize() {
    intake.set_brake_mode(MOTOR_BRAKE_COAST);
    indexer.set_brake_mode(MOTOR_BRAKE_COAST);
    ejector.set_brake_mode(MOTOR_BRAKE_COAST);
	pros::Task jamHandler(indexerJamHandler); 
    pros::Task colorSorting(colorSortingHandler);
}

void indexerJamHandler(void* pParam) {
    unsigned long now = pros::millis();
	int stallTime = -1;
	while (1) {
		if (!intaking && ((fabs(indexer.get_actual_velocity()) < 10 && indexer.get_target_velocity() > 0) ||
            (fabs(intake.get_actual_velocity()) < 10 && intake.get_target_velocity() > 0)))
            {
        // if (!intaking && (fabs(indexer.get_actual_velocity()) < 10 && indexer.get_target_velocity() > 0)) {
			if (stallTime == -1) {
				stallTime = pros::millis();
			} 
            else if (pros::millis() - stallTime >= 150) {
				indexerMutex.take(TIMEOUT_MAX);
                if (outtaking) {
                    intake.move(127);
                    indexer.move(127);
                    pros::delay(300);
                    intake.move(-127);
                    indexer.move(-127);
                }
                else {
                    intake.move(-127);
                    indexer.move(-127);
                    pros::delay(300);
                    intake.move(127);
                    indexer.move(127);
                }
	
				stallTime = -1;
				indexerMutex.give();
			}
		} else {
			stallTime = -1;
		}
        pros::Task::delay_until(&now, 5);
	}
}

void colorSortingHandler(void* pParam) {
    unsigned long now = pros::millis();
    colorSensor.set_led_pwm(25);
    while (1) {
        if ((blueAlliance && colorSensor.get_hue() < 30) || (redAlliance && colorSensor.get_hue() > 100)) {
            indexerMutex.take(TIMEOUT_MAX);
            if (midScore) {
                ejector.move(90);
            }
            else {
                ejector.move(-90);
            }
            pros::delay(250);
            if (midScore) {
                ejector.move(-90);
            }
            else {
                ejector.move(90);
            }
            indexerMutex.give();
        }

        pros::Task::delay_until(&now, 5);
    }
}

void intakeControlHandler(void* pParam) {
    unsigned long now = pros::millis();
    while (1) {
        if (controller.get_digital(DIGITAL_L1)) { // intake
            intakeBlocks(127);
        }
        else if (controller.get_digital(DIGITAL_A)) { // outtake
            lowScore(80);
        }
        else if (controller.get_digital(DIGITAL_R1)) { // mid score
            scoreMid(80);
        }
        else if (controller.get_digital(DIGITAL_R2)){ // high score
            scoreHigh(100);
        }
        else {
            stopIntake();
        }
        pros::Task::delay_until(&now, 20);
    }
}

void stopIntake() {
	indexerMutex.take(TIMEOUT_MAX); 
    intaking = false;
    midScore = false; 
    outtaking = false;
    intake.move(0);
    indexer.move(0);
    ejector.move(0);
	indexerMutex.give();
}

void intakeBlocks(int speed) {  
	indexerMutex.take(TIMEOUT_MAX);   
    intaking = true;
    intake.move(speed);
    indexer.move(speed - 50);
    ejector.move(0);
	indexerMutex.give();
}

void lowIntaking(int speed) {
    indexerMutex.take(TIMEOUT_MAX);   
    intaking = true;
    intake.move(speed);
    indexer.move(0);
    ejector.move(0);
	indexerMutex.give();
}

void lowScore(int speed) {
    indexerMutex.take(TIMEOUT_MAX);
    outtaking = true;
    intake.move(-speed);
    indexer.move(-speed);
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
    ejector.move(speed+30);
	indexerMutex.give();
}
