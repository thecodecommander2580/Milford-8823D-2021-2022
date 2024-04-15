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
// forkLift             motor         10              
// lift                 motor         20              
// liftPotentiometer    potV2         A               
// Inertial             inertial      1               
// fLiftPotentiometer   potV2         B               
// leftRotation         rotation      19              
// rightRotation        rotation      18              
// backRotation         rotation      17              
// PotentiometerG       pot           G               
// claw                 digital_out   C               
// frontLeftBase        motor         11              
// centerLeftBase       motor         12              
// backLeftBase         motor         13              
// frontRightBase       motor         14              
// centerRightBase      motor         15              
// backRightBase        motor         16              
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
//int slipclawSpeed = 95;


float liftUpperLimit = 297; //Configure the min and max height for the fork lift
float liftLowerLimit = 170;

float forkLiftUpperLimit = 325;
float forkLiftLowerLimit = 153;

float wheelDiamater = 4;


bool debug = false;

const double pi = 3.1415926;

// Functions

void moveBaseForward(double Distance, bool doNotContinue)
{

  double wheelCir = 2*pi*(wheelDiamater/2);

  double wheelRotations = Distance/wheelCir;

  frontLeftBase.rotateFor(reverse, wheelRotations, turns, false);
  centerLeftBase.rotateFor(reverse, wheelRotations, turns, false);
  backLeftBase.rotateFor(reverse, wheelRotations, turns, false);

  frontRightBase.rotateFor(reverse, wheelRotations, turns, false);
  centerRightBase.rotateFor(reverse, wheelRotations, turns, false);
  backRightBase.rotateFor(reverse, wheelRotations, turns, doNotContinue);

}

void moveBaseBackwards(double Distance, bool doNotContinue)
{

  double wheelCir = 2*pi*(wheelDiamater/2);

  double wheelRotations = Distance/wheelCir;

  frontLeftBase.rotateFor(forward, wheelRotations, turns, false);
  centerLeftBase.rotateFor(forward, wheelRotations, turns, false);
  backLeftBase.rotateFor(forward, wheelRotations, turns, false);

  frontRightBase.rotateFor(forward, wheelRotations, turns, false);
  centerRightBase.rotateFor(forward, wheelRotations, turns, false);
  backRightBase.rotateFor(forward, wheelRotations, turns, doNotContinue);

}

void setBaseSpeed(float speed)
{
  frontLeftBase.setVelocity(speed, percent);
  centerLeftBase.setVelocity(speed, percent);
  backLeftBase.setVelocity(speed, percent);

  frontRightBase.setVelocity(speed, percent);
  centerRightBase.setVelocity(speed, percent);
  backRightBase.setVelocity(speed, percent);
}

void resetMotorSpeeds() //Resets the motor speeds to their defaults as defined above.
{
  lift.setVelocity(liftSpeed, percent);
  forkLift.setVelocity(forkliftSpeed, percent);
  //slipClaw.setVelocity(slipclawSpeed, percent);
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
  frontLeftBase.setStopping(coast); //Configure Base Motors
  frontRightBase.setStopping(coast);
  centerLeftBase.setStopping(coast);
  centerRightBase.setStopping(coast);
  backLeftBase.setStopping(coast);
  backRightBase.setStopping(coast);
  frontLeftBase.setMaxTorque(100, percent);
  frontRightBase.setMaxTorque(100, percent);
  centerLeftBase.setMaxTorque(100, percent);
  centerRightBase.setMaxTorque(100, percent);
  backLeftBase.setMaxTorque(100, percent);
  backRightBase.setMaxTorque(100, percent);
  

  lift.setVelocity(liftSpeed, percent);  //Configure Lift Motor
  lift.setStopping(hold);
  lift.setMaxTorque(100, percent);

  forkLift.setVelocity(forkliftSpeed, percent);  //Configure Fork Lift Motor
  forkLift.setMaxTorque(100, percent);
  forkLift.setStopping(hold);

  //slipClaw.setStopping(hold);   //Configure Slip Claw
  //slipClaw.setMaxTorque(100, percent);
  //slipClaw.setVelocity(slipclawSpeed, percent);

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

     frontLeftBase.stop();
     frontRightBase.stop();
     drivebackPD(48, 100);
     frontLeftBase.spin(directionType::rev, 10, velocityUnits::pct);
     frontRightBase.spin(directionType::rev, 10, velocityUnits::pct);
     closeClaw();
     frontLeftBase.stop(brake);
     frontRightBase.stop(brake);
     drivePD(42, 100);
     while(fLiftPotentiometer.angle(degrees) > forkLiftLowerLimit) //Move the forklift down untill it's fully down
      {
        forkLift.spin(reverse);
      }

      forkLift.stop();                             //Stop the forklift
/*
  setBaseSpeed(75);                           //Set the velocity of the motors
  slipClaw.setVelocity(100, percent);

  moveBaseForward(50, true);                  //Drive to the goal
  slipClaw.rotateFor(reverse, 0.87, turns);   //Grab the goal
  setBaseSpeed(100);                          //Increase the speed for driving away
  moveBaseBackwards(30, true);                //Move the robot back to claim the goal
  
  while(fLiftPotentiometer.angle(degrees) > forkLiftLowerLimit) //Move the forklift down untill it's fully down
  {
    forkLift.spin(reverse);
  }

  forkLift.stop();                             //Stop the forklift
  */
  resetMotorSpeeds();                          //Resets the motor speeds to their defaults for driver control
  
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

   frontLeftBase.spin(forward);  
   frontRightBase.spin(forward);
   centerLeftBase.spin(forward);  
   centerRightBase.spin(forward);
   backLeftBase.spin(forward);  
   backRightBase.spin(forward);

    if(reverseDriving)  //Driving Contorls. To flip them, change the "reverseDriving" variable at the top of the program, where global instances are defined.
    {
      frontLeftBase.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
      centerLeftBase.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
      backLeftBase.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);

      frontRightBase.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
      centerRightBase.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
      backRightBase.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
    }

    else
    {
      frontLeftBase.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
      centerLeftBase.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);
      backLeftBase.setVelocity(-Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);

      frontRightBase.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
      centerRightBase.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
      backRightBase.setVelocity(-Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
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

    /*
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
    */

   if(Controller1.ButtonUp.pressing())  //Claw Controls
    {
      claw.set(true);
    }

    else if (Controller1.ButtonDown.pressing())
    {
      claw.set(false);
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
      printf("liftPotentiometer = %f \n" , liftPotentiometer.angle(degrees));
      printf("liftPotentiometer = %f \n\n" , fLiftPotentiometer.angle(degrees));
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