#include "vex.h"
#include <Odom.h>


using namespace vex;

const double pi = 3.14159;
double globalX = 0; //starting X coordinate
double globalY = 24; //starting Y coordinate
double absAngle = 0;
double intAngle = 0;
const double distR = 1.5; //Distance from right sensor to center of robot
const double distB = 2; //Distance from back wheel to center of robot
int pos() //This will constantly be running in the background updating the position as the robot moves.
{
  //reduce number of variables after it works
  double prevLEncoder = 0;
  double LEncoder;
  double prevREncoder = 0;
  double REncoder;
  double prevBEncoder = 0;
  double BEncoder;
  double prevAngle = 0;
  double deltaL;
  double deltaR;
  double deltaB;
  double deltaAngle;
  double totalDeltaL;
  double totalDeltaR;
  double totalDeltaB;
  double avgAngle;
  double localX;
  double localY;
  double radius;
  double polarAngle;
  double deltaGlobalX;
  double deltaGlobalY;
  leftRotation.resetRotation();
  rightRotation.resetRotation();
  backRotation.resetRotation();
  Inertial.setRotation(intAngle, degrees);
  while(true)
  {
    //Step 1: Grab the current encoder values
    LEncoder = leftRotation.position(rev);
    REncoder = rightRotation.position(rev);
    BEncoder = backRotation.position(rev);
    //Step 2: Calculate inches traveled since previous iteration of the loop (circumfrence * rev) for both sides
    deltaL = (LEncoder-prevLEncoder)*3*pi;
    deltaR = (REncoder-prevREncoder)*3*pi;
    deltaB = (BEncoder-prevBEncoder)*3*pi;

    //Step 2.5: Update previous encoder values
    prevLEncoder = LEncoder;
    prevREncoder = REncoder;
    prevBEncoder = BEncoder;
    //Step 3: Calculate the absolute change in the encoder value since the last full reset of the robot. Convert to inches.
    //These are only ever used to calculate the angle so if the inertia sensor works this step can be eliminated
    totalDeltaL = LEncoder*4*pi;
    totalDeltaR = REncoder*4*pi;
    totalDeltaB = BEncoder*4*pi;
    //Step 4: Calculate the absolute angle using deltaTheta = startingTheta + (deltaL - deltaR)/(wheelDistanceL + wheelDistanceR)
    //absAngle = Inertial.rotation(degrees)*pi/180;
    absAngle = Inertial.rotation(degrees)*pi/180;
    
    //Steps 3 and 4 can probably be replaced by the inertia sensor as it is more accurate and easier to use
    //Need to convert degrees to radians because all the other equations use radians. rad = degrees * pi/180

    //Step 5: Calculate the change in angle using deltaTheta = currentTheta - prevTheta
    deltaAngle = absAngle - prevAngle;
    //Step 6: If deltaTheta is EXACTLY 0, the local offset is <deltaBack, deltaRight or deltaLeft (they are equal)>
    if(deltaAngle == 0)
    {
      localX = deltaB;
      localY = deltaR;
    }
    //Step 7: If deltaTheta is NOT 0, calculate the local coordinates of 2sin(deltaTheta/2) * <(deltaS/deltaTheta) + distanceS, (deltaR/deltaTheta) + distanceR>
    else
    {
      localX = 2*sin(deltaAngle/2)*((deltaB/deltaAngle)+distB);
      localY = 2*sin(deltaAngle/2)*((deltaR/deltaAngle)+distR);
    }
    //Convert to global
    avgAngle = prevAngle + deltaAngle/2;
    radius = sqrt((localX*localX)+(localY*localY));
    if(localX != 0)
    {
    polarAngle = atan2(localY, localX);
    }
    else
    {
      if(localY > 0)
      {
        polarAngle = pi/2;
      }
      else
      {
        polarAngle = -pi/2;
      }
    }
    deltaGlobalX = radius*sin(polarAngle-avgAngle);
    deltaGlobalY = radius*cos(polarAngle-avgAngle);

    //Step 10: FINALLY update the global coordinates <xglobal, yglobal> = <xprev, yprev> + <xrotated, yrotated>
    globalX = globalX+deltaGlobalX;
    globalY = globalY+deltaGlobalY;
    
    //Step 11: Update prevTheta
    prevAngle = absAngle;

    //Screen printing the x, y, and angle for testing purposes
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.clearLine();
    Controller1.Screen.print("X value: %f", globalX);
    Controller1.Screen.setCursor(2, 0);
    Controller1.Screen.clearLine();
    Controller1.Screen.print("Y value: %f", globalY);
    Controller1.Screen.setCursor(4, 0);
    Controller1.Screen.clearLine();
    Controller1.Screen.print("Angle: %f", absAngle*180/pi);
    vex::wait(40, timeUnits::msec);
  }
  
  //This will constantly run in the background. It is up to me to decide when to pull down the most recent coordinates
}

void odomTurn(double targetO)
{
  float kP = 0.335;
  float kI = 0.0;
  float kD = 0.1;
  float error = 100;
  float totalError = 0;
  float changeError = 0;
  float prevError = 0;
  int speedCap = 30;
  float motorPower = 0;
  float currentAngle;
  while(fabs(error) > 1.5)
  {
    currentAngle = absAngle*180/pi;
    error = targetO - currentAngle;
    changeError = prevError-error;
    totalError = error+prevError;
    if(totalError > 500)
    {
      totalError = 500;
    }
    else if(totalError < -500)
    {
      totalError = -500;
    }
    else {}
    motorPower = kP*error+kI*totalError+kD*changeError;
    if(motorPower > speedCap)
    {
      motorPower = speedCap;
    }
    else if (motorPower < -speedCap)
    {
      motorPower = -speedCap;
    }
    else {}
    leftDrive.spin(directionType::fwd, motorPower, velocityUnits::pct);
    rightDrive.spin(directionType::fwd, -motorPower, velocityUnits::pct);
    prevError = error;
    vex::wait(40, msec);
  }
  
  leftDrive.stop(brake);
  rightDrive.stop(brake);

  if(absAngle < -2)
  {
    absAngle = absAngle+(2*pi);
  }
}
void turnToPoint(double desiredX, double desiredY)
{
  double yOff;
  double xOff;
  double desiredAngle;
  float rkP = 15;
  float error;
  float correction;
  yOff = desiredY-globalY;
  xOff = desiredX-globalX;
  if(xOff != 0)
  {
  desiredAngle = atan2(yOff, xOff);
  }
  else
  {
    if (yOff > 0)
    {
      desiredAngle = pi/2;
    }
    else
    {
      desiredAngle = -pi/2;
    }
  }
  while(fabs(desiredAngle-absAngle) > 0.0087)
  {
    error = desiredAngle-absAngle;
    if(error < 1)
    {
      correction = error*20;
    }
    correction = error*rkP;
    vex::wait(10, msec);
  }
    leftDrive.stop(brakeType::brake);
    rightDrive.stop(brakeType::brake);
}
void moveToPoint(double desiredX, double desiredY)
{
  //Step 1: turn to target
  float distToPoint = sqrt(pow(desiredX-globalX, 2)+pow(desiredY-globalY, 2));
  float intX = globalX;
  float intY = globalY;
  float intdistToPoint = distToPoint;
  float totalDist = 0;
  float prevError = 0;
  float totalError = 0;
  float changeError = 0;
  int speedCap = 60;
  float kP = 1.1;
  float kD = 0.3;
  float motorPower = 0;
  float OtoPoint = 0;
  //Turning to point adjustments:
  double yOff;
  double xOff;
  double desiredAngle;
  float rkP = 0.3;
  float Oerror;
  float correction;


  //Calculates the angle to point with the edge case of the x value difference being 0
  OtoPoint = atan2(desiredY-globalY, desiredX-globalX)*180/pi;
  odomTurn(OtoPoint);
  
  while(distToPoint > 0.5)
  {
    yOff = desiredY-globalY;
    xOff = desiredX-globalX;
    if(fabs(xOff) > 0.3)
    {
    desiredAngle = atan2(yOff, xOff);
      if(desiredAngle < 0 && absAngle > 0)
      {
        desiredAngle = desiredAngle+(2*pi);
      }
      else{
        //nothing
      }
    }
    else
    {
      if (yOff > 0)
      {
        desiredAngle = pi/2;
      }
      else
      {
        desiredAngle = -pi/2;
      }
    }
    Oerror = desiredAngle-absAngle;
    if (distToPoint > 0.5)
    {
      correction = Oerror*rkP;
    }
    else
    {
      correction = 0;
    }
    //distance to the target
    distToPoint = sqrt(pow(desiredX-globalX, 2)+pow(desiredY-globalY, 2));
    //distance traveled
    totalDist = sqrt(pow(intX-globalX, 2)+pow(intY-globalY, 2));
    //I-term garbage
    totalError += prevError;
    if(totalError > 500)
    {
      totalError = 500;
    }
    else if(totalError < -500)
    {
      totalError = -500;
    }
    //D-term garbage:
    changeError = distToPoint-prevError;
    prevError = distToPoint;
    //Error for the right side
    //Calculatin for the right correction
    //Calculation for the main speed
    motorPower = distToPoint*kP+changeError*kD;
    if(motorPower > speedCap)
    {
      motorPower = speedCap;
    }
    else
    {
      //nothing
    }
    if(totalDist > intdistToPoint) //if the robot traveles farther than it was supposed to: stop moving
    {
      break;
    }
    leftDrive.spin(directionType::fwd, motorPower+correction, velocityUnits::pct);
    rightDrive.spin(directionType::fwd, motorPower-correction, velocityUnits::pct);
    prevError = distToPoint;
    vex::wait(20, msec);
  }
    leftDrive.stop(brakeType::brake);
    rightDrive.stop(brakeType::brake);
}
void drivePD(double dist, int speedCap)
{
  const int diameter = 4;
  float kP = 1.3;
  float kD = 0.1;
  float error = dist;
  float prevError = dist;
  float derivError;
  float motorSpeed;
  float minSpeed = 35;
  leftDrive.setPosition(0,rev);
  rightDrive.setPosition(0, rev);
  while(leftDrive.position(rev)*diameter*pi < dist)
  {
    error = dist-(leftDrive.position(rev)*diameter*pi);
    derivError = prevError-error;
    motorSpeed = error*kP + derivError*kD;
    if(motorSpeed > speedCap)
    {
      motorSpeed = speedCap;
    }
    else if(motorSpeed < minSpeed)
    {
      motorSpeed = minSpeed;
    }
    leftDrive.spin(directionType::fwd, motorSpeed, velocityUnits::pct);
    rightDrive.spin(directionType::fwd, motorSpeed, velocityUnits::pct);
    Controller1.Screen.setCursor(2, 0);
    Controller1.Screen.clearLine();
    Controller1.Screen.print("Motor Speed: %f", motorSpeed);
    vex::wait(15, msec);
  }
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}
void drivebackPD(double dist, int speedCap)
{
  const int diameter = 4;
  float kP = 1.6;
  float kD = 0.1;
  float error = dist;
  float prevError = dist;
  float derivError;
  float motorSpeed;
  int minSpeed = 40;
  leftDrive.setPosition(0,rev);
  rightDrive.setPosition(0, rev);
  leftDrive.spin(directionType::rev, 20, velocityUnits::pct);
  rightDrive.spin(directionType::rev, 20, velocityUnits::pct);
  vex::wait(200, msec);
  while(fabs(leftDrive.position(rev)*diameter*pi) < dist)
  {
    error = dist-(fabs(leftDrive.position(rev)*diameter*pi));
    derivError = prevError-error;
    motorSpeed = error*kP + derivError*kD;
    if(motorSpeed > speedCap)
    {
      motorSpeed = speedCap;
    }
    else if(motorSpeed < minSpeed) 
    {
      motorSpeed = minSpeed;
    }
    else{}
    leftDrive.spin(directionType::rev, motorSpeed, velocityUnits::pct);
    rightDrive.spin(directionType::rev, motorSpeed, velocityUnits::pct);
    vex::wait(15, msec);
  }
  leftDrive.stop(brake);
  rightDrive.stop(brake);
}
void closeClaw()
{
  slipClaw.spin(directionType::rev, 100, velocityUnits::pct);
  vex::wait(1000, msec);
  slipClaw.stop(brake);
}