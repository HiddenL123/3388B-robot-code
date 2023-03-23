#include "display/lv_misc/lv_txt.h"
#include "drive.hpp"
#include "main.h"
#include "EZ-Template/util.hpp"
#include <cmath>


void Drive::go_to(double x,double y, int drive_speed, int turn_speed){
    double angle=atan(x/y)+pros::c::imu_get_heading(21);
    double distance=sqrt(pow(x,2)+pow(y,2));
    Drive::set_turn_pid(angle, turn_speed);
    Drive::wait_drive();
    Drive::set_drive_pid(distance, drive_speed);
    Drive::wait_drive();

}

void Drive::swerve_to(double x, double y, float end_angle, int speed){

}
void set_pure_pursuit(){

}
double calc_pursuit(float drive_posX, float targetX, float lookahead){
    float xValue=targetX-drive_posX;
    float radius=pow(lookahead,2)/(2*xValue);
    return(radius);
}
void Drive::pure_pursuit(float waypoints[][2],int waypointCount, float lookahead, float drive_speed, bool const){
    float initialX=Drive::x;
    float initialY=Drive::y;
    Drive::get_odometry();
    float driveposX=Drive::x;
    float driveposY=Drive::y;
    while(true){
        
        float shortestD=999999999.;
        int shortestP;
        for(int i=0;i<waypointCount;i++){
            float d=get_distance(waypoints[i][0],driveposX,waypoints[i][1],driveposY);
            
            if(d<shortestD){
                shortestD=d;
                shortestP=i;
            }
        }
        float x1;float x2;float y1; float y2;float heading;float dValue;
        float targetX;float targetY;
        float l=lookahead;

        bool endpoint=false;
        for(int i=shortestP;i<waypointCount;i++){
            float d=get_distance(waypoints[i][0],driveposX,waypoints[i][1],driveposY);
            if(d>lookahead){
                x1=waypoints[i-1][0];
                x2=waypoints[i][0];
                y1=waypoints[i-1][1];
                y2=waypoints[i][1];
                dValue=d;
                heading=pros::c::imu_get_heading(21);
                //uses sine and cosine law to calculate the exact position of the target point in between waypoints
                float d1=get_distance(x2,driveposX,y2,driveposY);//distance to waypoint 1
                float d2=get_distance(x1,driveposX,y1,driveposY);//distance to waypoint 2
                float d3=get_distance(x1,x2,y1,y2);//distance between waypoints

                float angle1=heading-atan((x1-driveposX)/(y1-driveposY));//angle to waypoint 1
                float angle2=heading-atan((x2-driveposX)/(y2-driveposY));//angle to waypoint 2
                float angle3=atan((x2-x1)/(y2-y1));//angle formed by line connecting waypoint

                float theta=abs(angle2-angle1);
                float sinbeta=(d2*sin(theta)/dValue);
                float delta=asin((lookahead*sinbeta)/d1);
                float alpha=180-(asin(sinbeta)+delta);
                float distanceA=sqrt(pow(d2,2)+pow(lookahead,2)+2*d1*lookahead*cos(alpha));
                
                //interpolates target position
                targetX=x1+distanceA+cos(angle3);
                targetY=y1+distanceA+sin(angle3);
                
                //transforms coordinates to robot position
        
                break;
            }else if(i==waypointCount-1){
                targetX=waypoints[i][0];
                targetY=waypoints[i][0];
                l=get_distance(targetX,driveposX,targetY,driveposY);
                endpoint=true;
            }
        }
        if(shortestD>l){
            targetX=waypoints[shortestP][0];
            targetY=waypoints[shortestP][0];
            l=get_distance(targetX,driveposX,targetY,driveposY);
        }
        float target_angle=heading-atan((targetX-driveposX)/(targetY-driveposY));
        float tx1=sin(target_angle)*l;
        float ty1=cos(target_angle)*l;

        double radius=calc_pursuit(driveposX, tx1, l);
        double leftRadius=radius+chassis_width/2;
        double rightRadius=radius-chassis_width/2;
        double rightVelocity=rightRadius/leftRadius;
        float rightM;
        float leftM;
        if(endpoint){drive_speed*=l/lookahead;}//goes slower as bot approches endpoints
        
        if(rightVelocity>1){
            rightM=drive_speed;
            leftM=drive_speed/rightVelocity;
        }else{
            rightM=drive_speed*rightVelocity;
            leftM=drive_speed;
        }
        
        
        set_tank(leftM,rightM);

    }
    
    

    Drive::get_odometry();
    Drive::x;
    Drive::y;

}


