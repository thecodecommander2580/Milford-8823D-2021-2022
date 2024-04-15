using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor forkLift;
extern motor lift;
extern potV2 liftPotentiometer;
extern inertial Inertial;
extern potV2 fLiftPotentiometer;
extern rotation leftRotation;
extern rotation rightRotation;
extern rotation backRotation;
extern pot autonSelect;
extern digital_out claw;
extern motor frontLeftBase;
extern motor centerLeftBase;
extern motor backLeftBase;
extern motor frontRightBase;
extern motor centerRightBase;
extern motor backRightBase;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );