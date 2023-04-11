#include "autons.hpp"
#include "EZ-Template/PID.hpp"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/adi.h"
#include "pros/imu.hpp"
#include "pros/motors.h"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}



// . . .
// Make your own autonomous functions here!
// . . .

long n=1;
PID cataPID{0.45, 0, 0, 0, "cataPID"};
PID intakePID{0.45, 0, 0, 0, "intakePID"};
void PIDsetup(){
  cataPID.set_exit_condition(80, 50, 300, 150, 500, 500);
  intakePID.set_exit_condition(80, 50, 300, 150, 500, 500);
}

void set_cata_PID(double target) {
  cataPID.set_target(target);
  ez::exit_output exit = ez::RUNNING;
  while (cataPID.exit_condition({incata_r}, true) == ez::RUNNING) {
    double output = cataPID.compute(incata_r.get_position());
    incata_r=output;
    incata_l=output;
    pros::delay(ez::util::DELAY_TIME);
  }
  incata_r=0;
  incata_l=0;
}

void cataLoad(){
  incata_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  incata_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while(!catalim.get_value()){
    incata_r=200;
    incata_l=200;
    pros::delay(ez::util::DELAY_TIME);
  }
  incata_r=0;
  incata_l=0;
}
void cataShoot(){
  incata_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  incata_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while(catalim.get_value()){
    incata_r=200;
    incata_l=200;
    pros::delay(ez::util::DELAY_TIME);
  }
  incata_r=0;
  incata_l=0;
}
void load(){
  if(!catalim.get_value()){
    incata_r=200;
    incata_l=200;
    pros::delay(ez::util::DELAY_TIME);
  }else{
    incata_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    incata_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    incata_r=0;
    incata_l=0;
  }
}
void shoot(){
  if(catalim.get_value()){
    incata_r=200;
    incata_l=200;
    pros::delay(ez::util::DELAY_TIME);
  }else{
    incata_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    incata_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    incata_r=0;
    incata_l=0;
  }
}




void roller_on(){
  incata_r=-200;
  incata_l=-200;
}
void roller_off(){
  incata_r=0;
  incata_l=0;
}
void intake_in(){

}
void intake_out(){

}
void left3disc(){
  roller_on();
  chassis.set_turn_pid(45, DRIVE_SPEED);
  chassis.wait_drive(load);
  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive();
  roller_off();
  chassis.set_turn_pid(-30, DRIVE_SPEED);
  chassis.wait_drive();
  cataShoot();

}
void leftlowgoal(){
  chassis.set_turn_pid(75, DRIVE_SPEED);
  chassis.wait_drive(load);
  
}
void autonLeft(){
  roller_on();
  chassis.set_drive_pid(4, DRIVE_SPEED, true);
  chassis.wait_drive(load);
  chassis.set_drive_pid(-16, DRIVE_SPEED, true);
  roller_off();;
  chassis.wait_drive(load);
  cataShoot();
}

void autonRight(){
  
}

void auton(){
  autonLeft();
  left3disc();
  leftlowgoal();
}










void skills_roller(){
  incata_r.move_relative(-500,200);
  incata_l.move_relative(-500,200);  
  pros::delay(800);
}
void matchLoad(int times){
  incata_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  incata_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  for(int i=0;i<times;i++){
    cataLoad();
    for(int j=0;j<300;j++){
      if(!catalim.get_value()){
        incata_r=200;
        incata_l=200;
      }else{
        incata_r=0;
        incata_l=0;
      }
      pros::delay(util::DELAY_TIME);
    }
    cataShoot();
  }
}

void skills(){
  PIDsetup();
  incata_r.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  incata_l.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  incata_r=0;
  incata_l=0;
  //from match loader
  //intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  chassis.set_turn_pid(-10,TURN_SPEED);
  chassis.wait_drive();

  matchLoad(3);

  chassis.set_turn_pid(10,TURN_SPEED);
  chassis.wait_drive();


  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-90,TURN_SPEED/2);
  chassis.wait_drive();

  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  skills_roller();//roller 1

  chassis.set_drive_pid(-25, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(0,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  skills_roller();//roller 2

  chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-45,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-200, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(190,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-65, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(170,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();

  //shoots
  matchLoad(3);

  chassis.set_turn_pid(190,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  skills_roller();//roller 3

  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(180,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  skills_roller();//roller 4

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(135, DRIVE_SPEED);
  chassis.wait_drive();
  bool on=false;
  while(1){
    launcher.set_value(!on);
  }
}
