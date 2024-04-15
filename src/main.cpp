/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\mab50                                            */
/*    Created:      Mon Jan 03 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftDrive            motor_group   21, 3           
// rightDrive           motor_group   9, 6            
// forkLift             motor         2               
// slipClaw             motor         11              
// lift                 motor         14              
// liftPotentiometer    potV2         A               
// leftRotation         encoder       C, D            
// rightRotation        encoder       E, F            
// backRotation         encoder       G, H            
// Inertial             inertial      4               
// fLiftPotentiometer   potV2         B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Odom.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

bool reverseDriving = false; //Changing this between "true" and "false" will flip the front and back of the robot when it comes to driving.

int liftSpeed = 90; //Configure the default speeds of motors
int forkliftSpeed = 95;
int slipclawSpeed = 95;


float liftUpperLimit = 297; //Configure the min and max height for the fork lift
float liftLowerLimit = 170;

float forkLiftUpperLimit = 325;
float forkLiftLowerLimit = 153;

bool debug = false;


// Functions

void resetMotorSpeeds() //Resets the motor speeds to their defaults as defined above.
{
  lift.setVelocity(liftSpeed, percent);
  forkLift.setVelocity(forkliftSpeed, percent);
  slipClaw.setVelocity(slipclawSpeed, percent);
}

/* //Unused Functions

void openSlipClaw(float speed) //Opens the Slip CLaw. Can use the default speed by setting it to 0. 
{
  float openPosition = 20; //The position the motor needs to rotate to open the Slip Claw.

  if(speed == 0) {speed = slipclawSpeed;} //Uses the default speed if the inputed speed is 0.
  
  slipClaw.resetPosition(); //Resets the posisition of the motor, then spins to "openPosition" at the provided speed.
  slipClaw.setVelocity(speed, percent);
  slipClaw.spinToPosition(openPosition, degrees, false);
}

void closeSlipClaw(float speed) //Closes the Slip Claw. Can use the default speed by setting it to 0. 
{
  float closePosition = 20; //The position the motor needs to rotate to close the Slip Claw.

  if(speed == 0) {speed = slipclawSpeed;} //Uses the default speed if the inputed speed is 0.
  
  slipClaw.resetPosition(); //Resets the posisition of the motor, then spins to "closePosition" at the provided speed.
  slipClaw.setVelocity(speed, percent);
  slipClaw.spinToPosition(closePosition, degrees, false);
}

void moveForkLift(float speed, float position) //Moves the Fork Lift. Can use the default speed by setting it to 0.
{
  if(speed == 0) {speed = forkliftSpeed;} //Uses the default speed if the inputed speed is 0.
  
  forkLift.setVelocity(speed, percent);
  forkLift.spinToPosition(position, degrees, false);
}

*/


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  leftDrive.setStopping(coast); //Configure Base Motors
  rightDrive.setStopping(coast);
  leftDrive.setMaxTorque(100, percent);
  rightDrive.setMaxTorque(100, percent);
  leftDrive.spin(forward);  
  rightDrive.spin(forward);

  lift.setVelocity(liftSpeed, percent);  //Configure Lift Motor
  lift.setStopping(hold);
  lift.setMaxTorque(100, percent);

  forkLift.setVelocity(forkliftSpeed, percent);  //Configure Fork Lift Motor
  forkLift.setMaxTorque(100, percent);
  forkLift.setStopping(hold);

  slipClaw.setStopping(hold);   //Configure Slip Claw
  slipClaw.setMaxTorque(100, percent);
  slipClaw.setVelocity(slipclawSpeed, percent);

  if(debug)
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.print("Debug");
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  leftDrive.stop();
  rightDrive.stop();

  while(fLiftPotentiometer.angle(degrees) > forkLiftLowerLimit)
  {
    forkLift.spin(reverse);
  }

  forkLift.stop();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {

    leftDrive.spin(forward); 
    rightDrive.spin(forward);

    if(reverseDriving)  //Driving Contorls. To flip them, change the "reverseDriving" variable at the top of the program, where global instances are defined.
    {
      leftDrive.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
      rightDrive.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
    }

    else
    {
      leftDrive.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
      rightDrive.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
    }

    if(Controller1.ButtonL1.pressing() && liftPotentiometer.angle(degrees) < liftUpperLimit)  //Lift Controls
    {
      lift.spin(forward);
    }

    else if(Controller1.ButtonL2.pressing() && liftPotentiometer.angle(degrees) > liftLowerLimit)
    {
      lift.spin(reverse);
    }

    else if(!Controller1.ButtonY.pressing())
    {
      lift.stop();
    }


    if(Controller1.ButtonR1.pressing() && fLiftPotentiometer.angle(degrees) < forkLiftUpperLimit) //Fork Lift Controls
    {
      forkLift.spin(forward);
    }

    else if(Controller1.ButtonR2.pressing() && fLiftPotentiometer.angle(degrees) > forkLiftLowerLimit )
    {
      forkLift.spin(reverse);
    }

    else if(!Controller1.ButtonY.pressing())
    {
      forkLift.stop();
    }


    if(Controller1.ButtonUp.pressing())  //Slip Claw Controls
    {
      slipClaw.spin(forward);
    }

    else if (Controller1.ButtonDown.pressing())
    {
      slipClaw.spin(reverse);
    }

    else
    {
      slipClaw.stop();
    }


    if(Controller1.ButtonL1.pressing() && Controller1.ButtonY.pressing())  //Fork Lift and Arm controls with an override with the "Y" button.
    {
      lift.spin(forward);
    }

    else if(Controller1.ButtonL2.pressing() && Controller1.ButtonY.pressing())
    {
      lift.spin(reverse);
    }

    else if(Controller1.ButtonY.pressing())
    {
      lift.stop();
    }


    if(Controller1.ButtonR1.pressing() && Controller1.ButtonY.pressing()) 
    {
      forkLift.spin(forward);
    }

    else if(Controller1.ButtonR2.pressing() && Controller1.ButtonY.pressing())
    {
      forkLift.spin(reverse);
    }

    else if(Controller1.ButtonY.pressing())
    {
      forkLift.stop();
    }

    if(debug)
    {
      //printf("liftPotentiometer = %f /n" , liftPotentiometer.angle(degrees));
      printf("liftPotentiometer = %f /n" , fLiftPotentiometer.angle(degrees));
    }
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}