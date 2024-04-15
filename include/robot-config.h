using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group leftDrive;
extern motor_group rightDrive;
extern motor forkLift;
extern motor slipClaw;
extern motor lift;
extern potV2 liftPotentiometer;
extern encoder leftRotation;
extern encoder rightRotation;
extern encoder backRotation;
extern inertial Inertial;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );