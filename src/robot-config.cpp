#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor forkLift = motor(PORT10, ratio36_1, false);
motor lift = motor(PORT20, ratio36_1, false);
potV2 liftPotentiometer = potV2(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT1);
potV2 fLiftPotentiometer = potV2(Brain.ThreeWirePort.B);
rotation leftRotation = rotation(PORT19, false);
rotation rightRotation = rotation(PORT18, false);
rotation backRotation = rotation(PORT17, false);
pot PotentiometerG = pot(Brain.ThreeWirePort.G);
digital_out claw = digital_out(Brain.ThreeWirePort.C);
motor frontLeftBase = motor(PORT11, ratio6_1, false);
motor centerLeftBase = motor(PORT12, ratio6_1, false);
motor backLeftBase = motor(PORT13, ratio6_1, false);
motor frontRightBase = motor(PORT14, ratio6_1, false);
motor centerRightBase = motor(PORT15, ratio6_1, false);
motor backRightBase = motor(PORT16, ratio6_1, false);

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