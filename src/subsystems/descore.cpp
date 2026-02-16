#include "main.h"

void setWing(bool stuff){
    if (stuff){
        wingPiston.extend();
    }
    else {
        wingPiston.retract();
    }
    
}

void wingPeriodic(){
    if (controller.get_digital_new_press(DIGITAL_A)){
        setWing(true);
    }   

    if (controller.get_digital_new_press(DIGITAL_B)){
        setWing(false);
    }
}