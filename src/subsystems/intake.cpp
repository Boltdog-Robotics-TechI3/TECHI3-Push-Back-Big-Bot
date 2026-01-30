#include "main.h"

bool loadOrLowForwards = true;
bool lowOrHighForwards = true;

void intakeInitialize() {
    intake1.set_brake_mode(MOTOR_BRAKE_COAST);
    intake2.set_brake_mode(MOTOR_BRAKE_COAST);
    //controller.clear();
    pros::delay(100);
    intakeStatus();
    pros::delay(100);

    
    intake1.set_current_limit(500);
    intake2.set_current_limit(500);
    midRoller.set_current_limit(500);
    colorSorter.set_current_limit(500);
    bottomRollers.set_current_limit(500);
    loadOrLow.set_current_limit(500);
    lowOrHigh.set_current_limit(500);
}

void intakePeriodic() {
	if (controller.get_digital(DIGITAL_L1)){ // intake
        hopper.move(0);
        lowOrHigh.move(0);
        setIntakeSpeed(127);
        hopper.move(0);
        lowOrHigh.move(0);
                

    }
    else if (controller.get_digital(DIGITAL_R1)){
        setIntakeSpeed(127);
    }
    

    /*if (controller.get_digital(DIGITAL_R1)){ // score high
        setIntakeSpeed(127);
        

    }*/

    else if (controller.get_digital(DIGITAL_L2)){ // outtake
        setIntakeSpeed(-127);
    }

    else if (controller.get_digital(DIGITAL_LEFT)){
        toggleLoadOrLow();
    }

    else if (controller.get_digital(DIGITAL_RIGHT)){
        togglelowOrHigh();
    }    

	else {
		setIntakeSpeed(0);

    }
}

void setIntakeSpeed(int speed){
    intake1.move(speed);
    intake2.move(speed);
    bottomRollers.move(speed);
    midRoller.move(speed);
    colorSorter.move(speed);
    hopper.move(speed);

    if (loadOrLowForwards) {
        loadOrLow.move(speed);
    }
    else { 
        loadOrLow.move(-speed);
    }

    if (lowOrHighForwards) {
        lowOrHigh.move(speed);
    }
    else { 
        lowOrHigh.move(-speed);
    }
}

void toggleLoadOrLow(){
    //if (controller.get_digital(DIGITAL_LEFT))
    if (loadOrLowForwards){setloadOrLowForwards(false);}
    else {setloadOrLowForwards(true);}
    intakeStatus();
    
}

void togglelowOrHigh(){
    //if (controller.get_digital(DIGITAL_RIGHT))
    if (lowOrHighForwards){setlowOrHighForwards(false);}
    else {setlowOrHighForwards(true);}
    intakeStatus();
    
    
}

bool getloadOrLowForwards(){
    return loadOrLowForwards;
}
void setloadOrLowForwards(bool stuff){
    loadOrLowForwards = stuff;
}

bool getlowOrHigh(){
    return lowOrHighForwards;
}

void setlowOrHighForwards(bool stuff){
    lowOrHighForwards = stuff;
}

void intakeStatus(){
    controller.print(0,0, "Front: %d Mid: %d", getloadOrLowForwards(),getlowOrHigh());
    pros::delay(100);
}