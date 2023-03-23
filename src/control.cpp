#include "main.h"
#include "global.hpp"
void control(){
    if (master.get_digital(DIGITAL_R1)){
      intake=200;
    }else if (master.get_digital(DIGITAL_R2)) {
      intake=-200;
    }else{
      intake=0;
    }
    if (master.get_digital(DIGITAL_UP)){
      cata=-200;
      cataOverride=true;
    }else if(!master.get_digital(DIGITAL_L1)&&!catalim.get_value()&&!cataOverride){
      cata=200;
    }else if (master.get_digital(DIGITAL_L1)&&!catalim.get_value()){
      cata=200;
    }else if(master.get_digital(DIGITAL_L2)&&catalim.get_value()){
      cata=200;
    }else if(master.get_digital(DIGITAL_DOWN)){
      cata=200;
      cataOverride=false;
    }else{
      cata.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      cata.move_velocity(0);
    }
    if(master.get_digital(DIGITAL_X)){
        piston_boost.set_value(true);
    }else{
        piston_boost.set_value(false);
    }
    if(master.get_digital(DIGITAL_A)){
      launcher.set_value(expansionExtend);
    }else if(master.get_digital(DIGITAL_B)){
      launcher.set_value(!expansionExtend);
    }/*else if(!master.get_digital(DIGITAL_A)&&elapsed>endTime+5){
      launcher.set_value(!on);
    }*/
    
    //printf("Time:"+elapsed);
}