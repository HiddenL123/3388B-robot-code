#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/imu.h"

void Drive::set_width(double width){
    double chassis_width=width;
}
void Drive::set_odometry(double x=0, double y=0, double angle=0){
    this->x=x;
    this->y=y;
    this->angle=angle;
    previousX=x;
    previousY=y;
    previous_heading=angle;
}

void Drive::get_odometry(){
    double left_rotation=TICK_PER_INCH*left_sensor();
    double right_rotation=TICK_PER_INCH*right_sensor();
    double wheel_angle=((left_rotation-prevLeft)-(right_rotation-prevRight))/chassis_width*180/M_PI;
    double imu_angle=pros::c::imu_get_heading(21)-previous_heading;
    double new_angle;
    double heading;
    //get the angle and heading
    if((wheel_angle-imu_angle)/imu_angle<0.01){
        new_angle=(wheel_angle+imu_angle)/2;
        heading=new_angle+previous_heading;
        previous_heading=heading;
        reset_gyro(heading);
    }else{
        imu_angle=heading;
        new_angle=imu_angle-previous_heading;
    }


    prevVFD=velocityFd;
    prevVSD=velocitySd;
    //d=2vt+1/2at^2

    double rad_angle=new_angle*M_PI/180;
    //gets the change in velocity (acceleration)
    pros::c::imu_accel_s_t accel=pros::c::imu_get_accel(21);
    double delta_right=(right_rotation-prevRight);
    double cA=(pow(prevVSD,2)+pow(velocityFd,2))/2/(delta_right/rad_angle);
    double af=accel.x/386.08858267717*sin(pros::c::imu_get_pitch(21));
    double as=accel.y/386.08858267717-cA;

    
    prevVFD=velocityFd;
    prevVSD=velocitySd;
    velocityFd+=af*util::DELAY_TIME;
    velocitySd+=as*util::DELAY_TIME;

    
    double fd_imu=(prevVFD+velocityFd)*util::DELAY_TIME*(sin(new_angle/2));
    
    double forwards_displacement=2*(delta_right/new_angle+chassis_width/2)*(sin(new_angle/2));
    double fd;
    if((forwards_displacement-fd_imu)/imu_angle<0.01){
        double fd=forwards_displacement;
    }else{
        fd=fd_imu;
    }

    double sd=(prevVSD+velocitySd)*util::DELAY_TIME*(sin(new_angle/2));

    x+=fd*cos(heading)+fd*sin(heading);
    y+=sd*cos(heading)+sd*sin(heading);

    

    
    prevRight=right_rotation;
    prevLeft=left_rotation;

}
void Drive::recalibrate_odometry(){
    velocityFd=0;
    velocitySd=0;
}
void recalibrateX();
void recalibrateY();
