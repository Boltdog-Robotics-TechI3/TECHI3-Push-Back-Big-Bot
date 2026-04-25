#include "main.h"

void setWing(bool extended){
    if (extended) {
        wingPiston.extend();
    }
    else {
        wingPiston.retract();
    }   
}

void setMatchLoader(bool extended) {
    if (extended) {
        matchLoadPiston.extend();
    }
    else {
        matchLoadPiston.retract();
    }
}

void setParkMech(bool extended) {
    if (extended) {
        matchLoadPiston.extend();
    }
    else {
        matchLoadPiston.retract();
    }
}

void pistonsPeriodic() {
    if (controller.get_digital_new_press(DIGITAL_L2)){
        setWing(true);
    }   
    if (controller.get_digital_new_release(DIGITAL_L2)){
        setWing(false);
    }

    if (controller.get_digital_new_press(DIGITAL_LEFT)) {
        setParkMech(!matchLoadPiston.is_extended());
    } 

    if (controller.get_digital(DIGITAL_L1) && controller.get_digital(DIGITAL_L2) &&
        controller.get_digital(DIGITAL_R1) && controller.get_digital(DIGITAL_R2)) {
        setParkMech(true);
    }
}