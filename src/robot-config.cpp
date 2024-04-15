#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftDriveMotorA = motor(PORT21, ratio18_1, true);
motor leftDriveMotorB = motor(PORT3, ratio18_1, true);
motor_group leftDrive = motor_group(leftDriveMotorA, leftDriveMotorB);
motor rightDriveMotorA = motor(PORT9, ratio18_1, false);
motor rightDriveMotorB = motor(PORT6, ratio18_1, false);
motor_group rightDrive = motor_group(rightDriveMotorA, rightDriveMotorB);
motor forkLift = motor(PORT2, ratio18_1, false);
motor slipClaw = motor(PORT11, ratio36_1, false);
motor lift = motor(PORT14, ratio36_1, false);
potV2 liftPotentiometer = potV2(Brain.ThreeWirePort.A);
encoder leftRotation = encoder(Brain.ThreeWirePort.C);
encoder rightRotation = encoder(Brain.ThreeWirePort.E);
encoder backRotation = encoder(Brain.ThreeWirePort.G);
inertial Inertial = inertial(PORT4);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}