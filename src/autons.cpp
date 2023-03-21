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



///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  if (chassis.interfered) {
   tug(3);
   return;
  }

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .
void basicAuton(){
  chassis.set_drive_pid(-2, DRIVE_SPEED, true);
  chassis.wait_drive();
  intake.move_relative(480, 200);


}
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
  while (cataPID.exit_condition({cata}, true) == ez::RUNNING) {
    double output = cataPID.compute(cata.get_position());
    cata=output;
    pros::delay(ez::util::DELAY_TIME);
  }
  cata=0;
}
void set_intake_PID(double target) {
  intakePID.set_target(target);
  ez::exit_output exit = ez::RUNNING;
  while (intakePID.exit_condition({intake}, true) == ez::RUNNING) {
    double output = intakePID.compute(intake.get_position());
    intake=output;
    pros::delay(ez::util::DELAY_TIME);
  }
  cata=0;
}
void cataLoad(){
  cata.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while(!catalim.get_value()){
    cata.move(200);
    pros::delay(ez::util::DELAY_TIME);
  }
  cata=0;
}
void cataShoot(){
  while(catalim.get_value()){
    cata=200;
    pros::delay(ez::util::DELAY_TIME);
  }
  cata.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cata=0;
}
void left1(){
  cataLoad();
  cataShoot();
  cataLoad();
  intake=200;
  pros::delay(500);

  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move_relative(480,200);
  pros::delay(1000);

  intake.move_relative(160, 200);
  chassis.set_drive_pid(-12, DRIVE_SPEED, true);

  cataShoot();
}
void left2(){//high goals
  cataLoad();
  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move_relative(-480,200);
  pros::delay(1000);

  chassis.set_drive_pid(-12, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-80, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, DRIVE_SPEED);
  chassis.wait_drive();

  cataShoot();
}
void left3(){
  cataLoad();

  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move_relative(-480,200);
  pros::delay(1000);

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();

  cataLoad();
  cataShoot();
}

void autonLeft(){
  left3();
}
void right1(){
  cataLoad();

  chassis.set_drive_pid(36, DRIVE_SPEED);//drives to roller
  chassis.wait_drive();

  cataLoad();
  cataShoot();//shoots

  chassis.set_turn_pid(90,TURN_SPEED);//turn
  chassis.wait_drive();
  
  chassis.set_drive_pid(18, DRIVE_SPEED);//drive to roller
  chassis.wait_drive();

  intake.move_relative(-480,200);//spin the roller
  pros::delay(1000);
}
void autonRight(){
  right1();
}
void autonRightHigh(){
  cataLoad();
  chassis.set_drive_pid(36, DRIVE_SPEED);//drives to roller
  chassis.wait_drive();

  chassis.set_turn_pid(90,TURN_SPEED);//turn
  chassis.wait_drive();
  
  chassis.set_drive_pid(18, DRIVE_SPEED);//drive to roller
  chassis.wait_drive();

  intake.move_relative(-480,200);//spin the roller

  chassis.set_drive_pid(12, DRIVE_SPEED);//drive to center
  chassis.wait_drive();

  chassis.set_turn_pid(-45,TURN_SPEED);//turn
  chassis.wait_drive();

  chassis.set_drive_pid(80, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(90,TURN_SPEED);//turn
  chassis.wait_drive();

  cataShoot();
  
  pros::delay(1000);
}
void combinedAuton(){
  cataLoad();

  chassis.set_drive_pid(6, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move_relative(-480,200);
  pros::delay(900);

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();

  cataLoad();
  cataShoot();

  chassis.set_turn_pid(-135,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(200, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(90,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move_relative(-480,200);
  pros::delay(800);

}
void roller(){
  intake.move_relative(500,200);
  pros::delay(800);
}
void matchLoad(int times){
  cata.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  for(int i=0;i<times;i++){
    cataLoad();
    for(int j=0;j<300;j++){
      if(!catalim.get_value()){
        cata=200;
      }else{
        cata=0;
      }
      pros::delay(util::DELAY_TIME);
    }
    cataShoot();
  }
}

void skills(){
  PIDsetup();
  cata.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cata=0;
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

  roller();//roller 1

  chassis.set_drive_pid(-25, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(0,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  roller();//roller 2

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

  roller();//roller 3

  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(180,TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  roller();//roller 4

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(135, DRIVE_SPEED);
  chassis.wait_drive();
  bool on=false;
  while(1){
    launcher.set_value(!on);
  }
}