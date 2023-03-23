#include "EZ-Template/util.hpp"
#include "main.h"

void Drive::setGoalLocation(float x, float y){
    float goalX=x;
    float goalY=y;
}
void Drive::setRange(float min, float max){
    minRange=min;
    maxRange=max;
}
void Drive::aimbot_angle(){
    double angle=atan((goalX-x)/(goalY-y));
    set_turn_pid(angle, 100);
    wait_drive();
    
}
void Drive::aimbot_angle_and_shoot(void (*shoot)()){
    aimbot_angle();
    float d=get_distance(x,goalX , y, goalY);
    if(d>maxRange){
        set_drive_pid((d-maxRange)*1.1,100);
        wait_drive();
    }else if(d<minRange){
        set_drive_pid((minRange-d)*1.1,100);
        wait_drive();
    }
    shoot();
}