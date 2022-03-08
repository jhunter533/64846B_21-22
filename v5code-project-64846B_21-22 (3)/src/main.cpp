/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jessica Hunter                                            */
/*    Created:      Wed Sep 8 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// Back                 motor         5
// Claw                 motor         10
// FrontLeft            motor         6
// BackLeft             motor         13
// FrontRight           motor         12
// BackRight            motor         2
// Lift                 motor_group   9,1
// Controller2          controller
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"
using namespace vex;

motor_group LeftDriveSmart = motor_group(FrontLeft, BackLeft);

motor_group RightDriveSmart = motor_group(FrontRight, BackRight);

smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1);

//////////////PID Turning////////////////////////////////////////////////////////
// PID = Porportion, Inegral, Deriviative (Tuning Parameters)
// Porportion: Distance to target angle
// Inegral: Acumulated error within the threshold
// Derivative: Rate of change of the error (Note: not needed for our purposes)

// Guidelines for Tuning:
//    - First: Baseline your parameters to learn the right values for Your robot
//        - set kP = 0.1 (this seems to be a good place to start)
//        - set kI and kD to 0 (noting kD will not change)
//        - set maxSpeed (noting the higher maxSpeed is the greater the initial
//        rotation,
//                          but may be more erratic)
//        - set turnThreshold = 2 (something small enough to learn the
//        appropriate threshold)
//        - set turnTolerance = 1 (something reasonable that says we are close
//        enough)
//              (note: turnThreshold > turnTolerance, and they represent
//              boundaries on the error)
//        - set maxIter to something reasonably high (~ something in the 100s)
//    - Next: Learn the correct turnThreshold for Your Robot!!
//        - Note: at the end of the while loop, make sure the following is all
//        uncommented
//            - final error and derivative calculations
//            - print statements for iter, error, and derivative
//        - !! Download and Run !!
//            - We expect iter == maxIter (this means that the code terminated
//            without
//                meeting the expected turnTolerance.)
//        - Round up the error printed to the screen and set turnThreshold to
//        that value.
//        - Now start to slowly turn on kI <= kP
//        - !! Download and Run !!
//    - Finally: Tune kI
//        - If iter == maxIter
//            - increase kI
//        - If error < 0
//            - decrease kI
//********//Have fun tuning and learning how these parameters work
// together//********//

// Used to count the number of turns for output data only
int turnCount = 0;
// Relative degree tracking
int angleTracker = 0;
// Weighted factor of porportion error
double kP = 0.18;
// Weighted factor of integral error
double kI = 0.008;
// Weighted factor of the derivated error
double kD = .1;
// Max speed in Volts for motors
double maxSpeed = 6;
// The angle difference from error when integral adjustments turns on
int turnThreshold = 16;
// Tolerance for approximating the target angle
double turnTolerance = .5;
// Total number of iterations to exit while loop
int maxIter = 300;

// Turning Function
void turnPID(double angleTurn) {
  //  Distance to target in degrees
  double error = 0;
  //  Error degree from the last iteration
  double prevError = 0;
  // Derivative of the error. The slope between iterations
  double derivative = 0;
  // Accumulated error after threashold. Summing error
  double integral = 0;
  // Iterations of the loop. Counter used to exit loop if not converging
  double iter = 0;

  // Used for Relative Coordinates. For absolute coordinates, comment out
  // following lines for angleTracker
 // angleTracker += angleTurn;
  //angleTurn = angleTracker % 360;

  // Automated error correction loop
  while (fabs(TurnGyroSmart.rotation(degrees) - angleTurn) > turnTolerance && iter < maxIter) {
    iter += 1;
    error = angleTurn - TurnGyroSmart.rotation(degrees);
    derivative = error - prevError;
    prevError = error;

    // Checking if error passes threshold to build the integral
    if (fabs(error) < turnThreshold && error != 0) {
 integral += error;
    } else {
      integral = 0;
    }

    // Voltage to use. PID calculation
    double powerDrive = error * kP + derivative * kD + integral * kI;

    // Capping voltage to max speed
    if (powerDrive > maxSpeed) {
      powerDrive = maxSpeed;
    } else if (powerDrive < -maxSpeed) {
      powerDrive = -maxSpeed;
    }

    // Send to motors
    LeftDriveSmart.spin(forward, powerDrive, voltageUnits::volt);
    RightDriveSmart.spin(forward, -powerDrive, voltageUnits::volt);

    this_thread::sleep_for(15);
  }

  // Angle achieved, brake robot
  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);

  // Tuning data, output to screen
  turnCount += 1;
  error = angleTurn - TurnGyroSmart.rotation(degrees);
  derivative = error - prevError;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Turn #: %d", turnCount);
  Controller1.Screen.setCursor(1, 13);
  Controller1.Screen.print("iter: %.0f", iter);
  Controller1.Screen.newLine();
  Controller1.Screen.print("error: %.5f", error);
  Controller1.Screen.newLine();
  Controller1.Screen.print("derivative: %.5f", derivative);
  Controller1.Screen.newLine();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                   */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// Creates a competition object that allows access to Competition methods.
vex::competition Competition;
/*                  GLOBAL DEFINITIONS
 *
 *      These global variables are used across this program to maintain state
 * (choices)
 *
 */
// storage for our auton selection
int autonomousSelection = -1;

// collect data for on screen button and include off and on color feedback for
// button pric - instead of radio approach with one button on or off at a time,
// each button has a state. ie shoot Preload may be low yellow and high yellow when on.
typedef struct _button {
  int xpos;
  int ypos;
  int width;
  int height;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char *label;
} button;

// Button array definitions for each software button. The purpose of each button
// data structure is defined above. The array size can be extended, so you can
// have as many buttons as you wish as long as it fits.
button buttons[] = {
    {30, 30, 60, 60, false, 0xE00000, 0x00E000, "RMidOnly"},
    {150, 30, 60, 60, false, 0xE00000, 0x00E000, "L1Yellow"},
    {270, 30, 60, 60, false, 0xE00000, 0x00E000, "R1Yellow"},
    {390, 30, 60, 60, false, 0xE00000, 0x00E000, "R2Yellow"},
    {30, 150, 60, 60, false, 0xE00000, 0x00E000, "Blank"},
    {150, 150, 60, 60, false, 0xE00000, 0x00E000, "L2Yellow"},
    {270, 150, 60, 60, false, 0xE00000, 0x00E000, "Nothing"},
    {390, 150, 60, 60, false, 0xE00000, 0x00E000, "skills"}};

// forward ref
void displayButtonControls(int index, bool pressed);

/*-----------------------------------------------------------------------------*/
/** @brief      Check if touch is inside button */
/*-----------------------------------------------------------------------------*/

int findButton(int16_t xpos, int16_t ypos) {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    button *pButton = &buttons[index];
    if (xpos < pButton->xpos || xpos > (pButton->xpos + pButton->width))
      continue;
      
  //  if (xpos < pButton->xpos || xpos > (pButton->xpos + pButton->height))
      //continue;
    
    if (ypos < pButton->ypos || ypos > (pButton->ypos + pButton->height))
      continue;

    return (index);
  }
  return (-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Init button states */
/*-----------------------------------------------------------------------------*/

void initButtons() {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    buttons[index].state = false;
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been touched */
/*-----------------------------------------------------------------------------*/

void userTouchCallbackPressed() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    displayButtonControls(index, true);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been (un)touched */
/*-----------------------------------------------------------------------------*/

void userTouchCallbackReleased() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    // clear all buttons to false, ie. unselected
    //      initButtons();

    // now set this one as true
    if (buttons[index].state == true) {
      buttons[index].state = false;
    } else {
      buttons[index].state = true;
    }

    // save as auton selection
    autonomousSelection = index;

    displayButtonControls(index, false);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Draw all buttons */
/*-----------------------------------------------------------------------------*/

void displayButtonControls(int index, bool pressed) {
  vex::color c;
  Brain.Screen.setPenColor(vex::color(0xe0e0e0));

  for (int i = 0; i < sizeof(buttons) / sizeof(button); i++) {

    if (buttons[i].state)
      c = buttons[i].onColor;
    else
      c = buttons[i].offColor;

    Brain.Screen.setFillColor(c);

    // button fill
    if (i == index && pressed == true) {
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
          buttons[i].width, buttons[i].height, c);
    } else
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
         buttons[i].width, buttons[i].height);

    // outline
    Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
         buttons[i].width, buttons[i].height,
        vex::color::transparent);

    // draw label
    if (buttons[i].label != NULL)
      Brain.Screen.printAt(buttons[i].xpos + 8,
       buttons[i].ypos + buttons[i].height - 8,
        buttons[i].label);
  }
}

using namespace vex;

// define your global instances of motors and other devices here

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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

// Autonomous function opns
void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  /* initialize capabilities from buttons */

  // Bool statements for when code is running to press before competition starts
  bool RMidOnly = buttons[0].state;
  bool Skills = buttons[7].state;
  bool R1Yellow = buttons[2].state;
  bool R2Yellow = buttons[3].state;
  bool L1Yellow = buttons[1].state;
  bool L2Yellow = buttons[5].state;
  bool  Nothing= buttons[6].state;
  bool Blank = buttons[4].state;

  if (Skills) {

    //..........Starting Skills on Red Left..........//
    // Set stopping functions for rest of the code


  //skills
  
  Drivetrain.setStopping(brake);
  Claw.setStopping(brakeType::hold);
  Back.setStopping(brakeType::hold);
//lower back lift
  Back.spinFor(forward,550,degrees,80,velocityUnits::pct);
  //drive enough for back lift to be under goal to starting position
  Drivetrain.driveFor(reverse,12,inches,70,velocityUnits::pct);
  //Drivetrain.stop(brake);
  wait(.3,sec);
  //wait to ensure it is on lift
Back.spinFor(reverse,440,degrees,80,velocityUnits::pct);
//pick up goal on platform with the back
//turn to head to left yellow goal
turnPID(83);
//turnPID(87);
//drive forward then turn to allow claw to face goal
Drivetrain.driveFor(forward,24,inches,90,velocityUnits::pct);
turnPID(7+86);
//turnPID(8.75);
//drive till the yellow goal is reached
Drivetrain.driveFor(forward,23.3,inches,60,velocityUnits::pct);


//Drivetrain.driveFor(forward,49,inches,90,velocityUnits::pct);
//Drivetrain.stop(brake);
//wait to avoid overshooting
wait(.2,sec);
//spin claw and lift to pick up left yellow goal
Claw.spinFor(reverse,130,degrees,80,velocityUnits::pct);
Lift.spinFor(reverse,200,degrees,80,velocityUnits::pct);
//drive forward holding goal
Drivetrain.driveFor(forward,32,inches,90,velocityUnits::pct);
//turn to start reaching platform
turnPID(93+40);
//drive forward
//robot should be in front of platform at an angle
Drivetrain.driveFor(forward,14,inches,80,velocityUnits::pct);
//spin claw to get a better grip as we raise the lift.
Claw.spinFor(reverse,40,degrees,60,velocityUnits::pct,false);
Lift.spinFor(reverse,900,degrees,80,velocityUnits::pct,true);
//drive closer to the goal
Drivetrain.driveFor(forward,5,inches,40,velocityUnits::pct);
//turn to be parallel with platform
turnPID(179);
//drive forward so the front wheels are in line with platform stand
Drivetrain.driveFor(forward,9.3,inches,60,velocityUnits::pct);
//wait to stop drift
wait(.2,sec);
//spin only the right side of chassis
//to slide in between the black platform stand 
RightDriveSmart.spinFor(forward,860,degrees,60,velocityUnits::pct);
//lower lift and release claw
Lift.spinFor(forward,400,degrees,90,velocityUnits::pct);
Claw.spinFor(forward,100,degrees,90,velocityUnits::pct);
Drivetrain.driveFor(reverse,4,inches,60,velocityUnits::pct);
//reverse enough to drop goal
//raise lift to get over platform edge
Lift.spinFor(reverse,50,deg,80,velocityUnits::pct);
Drivetrain.driveFor(reverse,5,inches,60,velocityUnits::pct);
//back out of platform
Claw.spinFor(forward,40,deg,90,velocityUnits::pct,false);
Lift.spinFor(forward,700,degrees,90,velocityUnits::pct);
//return lift and claw to starting position

turnPID(180);
//turn to right 90
Back.spinFor(forward,450,degrees,90,velocityUnits::pct);
//lower the back
Drivetrain.driveFor(forward,15,inches,70,velocityUnits::pct);
Back.spinFor(reverse,400,deg,80,velocityUnits::pct);
//drop goal in back lift and drive forward
//turn to face goal with claw
turnPID(0);
//drive to reach alliance goal 
Drivetrain.driveFor(forward,14,inches,70,velocityUnits::pct);
//pick up the goal and raise it
Claw.spinFor(reverse,120,deg,80,velocityUnits::pct);
Lift.spinFor(reverse,900,deg,80,velocityUnits::pct);
turnPID(60);
//turn to platform
Drivetrain.driveFor(forward,8,inches,40,velocityUnits::pct);
turnPID(105);
Drivetrain.driveFor(forward,2,inches,40,velocityUnits::pct);
//drive forward then turn and drive again to push neutral goal toward side
Lift.spinFor(forward,300,deg,80,velocityUnits::pct);
Claw.spinFor(forward,100,degrees,90,velocityUnits::pct);
Drivetrain.driveFor(reverse,2,inches,60,velocityUnits::pct);
Lift.spinFor(reverse,150,deg,80,velocityUnits::pct);
Drivetrain.driveFor(reverse,5,inches,60,velocityUnits::pct);
Claw.spinFor(forward,40,deg,90,velocityUnits::pct,false);
Lift.spinFor(forward,750,degrees,90,velocityUnits::pct);
//drop goal and return lift to position
turnPID(180);
Drivetrain.driveFor(reverse,12,inches,60,velocityUnits::pct,false);
//turn to alliance corner goal
//Back.spinFor(forward,400,deg,70,velocityUnits::pct);
Back.spinFor(forward,400,deg,70,velocityUnits::pct,true);
Drivetrain.driveFor(reverse,28,inches,60,velocityUnits::pct);
Back.spinFor(reverse,400,deg,90,velocityUnits::pct);
//back into the corner goal

//Claw.spinFor(reverse,140,deg,80,velocityUnits::pct);
//Lift.spinFor(reverse,40,degrees,90,velocityUnits::pct);
turnPID(180+30);

Drivetrain.driveFor(forward,50,inches,60,velocityUnits::pct);
Claw.spinFor(reverse,100,deg,90,velocityUnits::pct);
Drivetrain.driveFor(forward,50,inches,60,velocityUnits::pct);
//drive into tall neutral goal pushing it
turnPID(240);
Drivetrain.driveFor(forward,22,inches,60,velocityUnits::pct);
//turn to push out of way of corner alliance goal
Claw.spinFor(forward,130,deg,90,velocityUnits::pct);
Drivetrain.driveFor(reverse,12,inches,60,velocityUnits::pct);
turnPID(270);
Back.spinFor(forward,400,deg,70,velocityUnits::pct);
Drivetrain.driveFor(forward,10,inches,60,velocityUnits::pct);
turnPID(90);
Back.spinFor(reverse,400,deg,70,velocityUnits::pct);
Drivetrain.driveFor(forward,12,inches,60,velocityUnits::pct);
Claw.spinFor(reverse,120,deg,90,velocityUnits::pct);
Lift.spinFor(reverse,20,degrees,90,velocityUnits::pct);
turnPID(-30);


    //...............END OF CODE...............//
  }

  if (L1Yellow) {

    //..........Starting Blue Left..........//

    // Set stopping functions for the rest of the code
    Drivetrain.setStopping(brake);
    Claw.setStopping(brakeType::hold);
    Back.setStopping(brakeType::hold);
      Back.spinFor(forward,550,degrees,100,velocityUnits::pct,false);
  //spin back u lift to resting on the ground
  wait(.2,sec);
  //wait so the lift is down far enough before driving
Drivetrain.driveFor(reverse,50,inches,100,velocityUnits::pct);
//drive towards the goal


//ignore
//Claw.spinFor(reverse,130,degrees,80,velocityUnits::pct);

//auton left yellow middle
//include auton left yellow only
//spin back u lift up so the goal is nestled
  Back.spinFor(reverse,300,degrees,80,velocityUnits::pct);
  //drive back to start
  Drivetrain.driveFor(forward,50,inches,100,velocityUnits::pct);


    //..........Prepare for User Control..........//
  }

  if (L2Yellow) {

    //..........Starting Red Right..........//
    Drivetrain.setStopping(brake);
    Claw.setStopping(brakeType::hold);
    Back.setStopping(brakeType::hold);
    // Set stopping functions for the rest of the code
     Back.spinFor(forward,550,degrees,100,velocityUnits::pct,false);
  //spin back u lift to resting on the ground
  wait(.2,sec);
  //wait so the lift is down far enough before driving
Drivetrain.driveFor(reverse,50,inches,100,velocityUnits::pct);
//drive towards the goal


//ignore
//Claw.spinFor(reverse,130,degrees,80,velocityUnits::pct);

//auton left yellow middle
//include auton left yellow only
//spin back u lift up so the goal is nestled
  Back.spinFor(reverse,300,degrees,80,velocityUnits::pct);
  //drive back to start
 // Drivetrain.driveFor(forward,50,inches,100,velocityUnits::pct);
  //dont include in left mid above one lines
  
  //from picking up left neutral goal
  //turn to face an angle to avoid rings
  turnPID(-84);
  //drive most of  the way to the middle neutral goal
  Drivetrain.driveFor(forward,15,inches,100,velocityUnits::pct);
  //turn to face the goal
  turnPID(-130);
  wait(.2,sec);
  //drive to goal
Drivetrain.driveFor(forward,5,inches,60,velocityUnits::pct);
//wait to make sure no skiding
wait(.2,sec);
//clamp the claw down 
Claw.spinFor(reverse,140,degrees,80,velocityUnits::pct);
//turn with goal
turnPID(10);
//drive back to start
Drivetrain.driveFor(forward,30,inches,90,velocityUnits::pct);

    //..........Prepare for User Control..........//
  }

  if (R1Yellow) {

    //..........Starting Red Left..........//
        Drivetrain.setStopping(brake);
    Claw.setStopping(brakeType::hold);
    Back.setStopping(brakeType::hold);

    // Set stopping functions for the rest of the code
     // Back.spinFor(forward,550,degrees,100,velocityUnits::pct,false);
 // wait(.2,sec);
 
Drivetrain.driveFor(forward,43,inches,100,velocityUnits::pct);
wait(.2,sec);
Claw.spinFor(reverse,130,degrees,80,velocityUnits::pct);
Drivetrain.driveFor(reverse,42,inches,100,velocityUnits::pct);

    //..........Prepare for User Control..........//
  }

  if (R2Yellow) {

    //..........            ..........//
        Drivetrain.setStopping(brake);
    Claw.setStopping(brakeType::hold);
    Back.setStopping(brakeType::hold);


Drivetrain.driveFor(forward,43,inches,90,velocityUnits::pct);
wait(.2,sec);
Claw.spinFor(reverse,130,degrees,80,velocityUnits::pct);
Lift.spinFor(reverse,30,degrees,90,velocityUnits::pct);
Drivetrain.driveFor(reverse,12,inches,100,velocityUnits::pct);
turnPID(120);
Claw.spinFor(forward,130,degrees,100,velocityUnits::pct);
Back.spinFor(forward,550,deg,90,velocityUnits::pct);

Drivetrain.driveFor(reverse,32,inches,100,velocityUnits::pct);
Back.spinFor(reverse,200,deg,90,velocityUnits::pct);
wait(.2,sec);
Drivetrain.driveFor(forward,14,inches,100,velocityUnits::pct);
turnPID(160);
Drivetrain.driveFor(forward,40,inches,100,velocityUnits::pct);
    //..........Prepare for User Control..........//
  }

  if (RMidOnly) {

    //..........        ..........//
        Drivetrain.setStopping(brake);
    Claw.setStopping(brakeType::hold);
    Back.setStopping(brakeType::hold);


Drivetrain.driveFor(forward,18,inches,100,velocityUnits::pct);
turnPID(-38);
Drivetrain.driveFor(forward,38.5,inches,100,velocityUnits::pct);

wait(.2,sec);
Claw.spinFor(reverse,130,degrees,80,velocityUnits::pct);
Drivetrain.driveFor(reverse,40,inches,100,velocityUnits::pct);
    //..........Prepare for User Control..........//
  }

  if (Blank) {

    //..........Starting Skills on Blue Left..........//

    // Set stopping functions for the rest of the code


    //...............END OF CODE...............//
  }

  if (Nothing) {

    //..........Starting Blue Right..........//

    // Set stopping functiions for the rest of the code

    //..........Prepare for User Control..........//
  }
}

  //...............END OF CODE...............//


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*                                                                           */
/*---------------------------------------------------------------------------*/
bool halfspeed = false;
//bool Lhalfspeed = false;
void LiftReset() {
  Lift.spinTo(-1000,degrees,100,velocityUnits::pct);
}
void halfspeedcontrol() { halfspeed = !halfspeed; }
//void LiftSpeed () { Lhalfspeed = !halfspeed;}
//bool qspeed = false;

//void qspeedcontrol() {qspeed = !qspeed;}
void usercontrol(void) {

  int threshold = 20, ChasLfVar = 0, ChasRtVar = 0;
  Controller1.ButtonA.pressed(halfspeedcontrol);
//Controller2.ButtonA.pressed(LiftSpeed);
 // Controller1.ButtonB.pressed(qspeedcontrol);
  

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // TestAuton Callback Function //
  Controller1.ButtonA.pressed(halfspeedcontrol);
   //Controller2.ButtonA.pressed(LiftSpeed);


    // Tank Drivetrain //

    // Left Side Chassis

    if (abs(Controller1.Axis3.position(percentUnits::pct)) > threshold) {
      ChasLfVar = Controller1.Axis3.position(percentUnits::pct);

    }

    else {
      ChasLfVar = 0;
      LeftDriveSmart.stop(brakeType::brake);
    }

    // Right Side Chassis

    if (abs(Controller1.Axis2.position(percentUnits::pct)) > threshold) {
      ChasRtVar = Controller1.Axis2.position(percentUnits::pct);
    }

    else {
      ChasRtVar = 0;
      RightDriveSmart.stop(brakeType::brake);
    }
    // halfspeed control
    if (halfspeed == true) {
            LeftDriveSmart.spin(directionType::fwd, ChasLfVar * .50,percentUnits::pct);
      RightDriveSmart.spin(directionType::fwd, ChasRtVar * .50, percentUnits::pct);

    } else {
      LeftDriveSmart.spin(directionType::fwd, ChasLfVar, percentUnits::pct);
      RightDriveSmart.spin(directionType::fwd, ChasRtVar, percentUnits::pct);
    }

    
    // Claw Controls //

    if (Controller2.ButtonR1.pressing()) {
      Claw.spin(directionType::fwd, 60, velocityUnits::pct);
    } else if (Controller2.ButtonR2.pressing()) {
      Claw.spin(directionType::rev, 60, velocityUnits::pct);
    } else {
      Claw.stop(brakeType::hold);
    }

    // Lift Controls //

    if (Controller2.ButtonL2.pressing()) {
      Lift.spin(directionType::fwd,75,velocityUnits::pct);
    } else if (Controller2.ButtonL1.pressing()) {
      Lift.spin(directionType::rev,75,velocityUnits::pct);
    } else {
      Lift.stop(brakeType::hold);
    }

    // Back Controls //

    if (Controller1.ButtonL1.pressing()) {
      Back.spin(directionType::fwd, 100, velocityUnits::pct);
    } else if (Controller1.ButtonR1.pressing()) {
      Back.spin(directionType::rev, 100, velocityUnits::pct);
    } else {
      Back.stop(brakeType::hold);
    }
  
  wait(100,msec);
}
}
// Main will set up the competition functions and callbacks

int main() {

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Run the pre-autonomous function
  pre_auton();

  // Set up callbacks for autonomous and driver control periods
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Register events for button selection
  Brain.Screen.pressed(userTouchCallbackPressed);
  Brain.Screen.released(userTouchCallbackReleased);

  // Make nice background
  Brain.Screen.setFillColor(vex::color(0x404040));
  Brain.Screen.setPenColor(vex::color(0x404040));
  Brain.Screen.drawRectangle(0, 0, 480, 120);
  Brain.Screen.setFillColor(vex::color(0x808080));
  Brain.Screen.setPenColor(vex::color(0x808080));
  Brain.Screen.drawRectangle(0, 120, 480, 120);

  // Initial Display
  displayButtonControls(0, false);
Controller1.Screen.print(BackRight.rotation(rotationUnits::raw));

//12951
//2.7w motor
  // While loop to call back functions to run during competition
  while (1) {
    // Allow other tasks to run
    if (!Competition.isEnabled())
      Brain.Screen.setFont(fontType::mono40);
    Brain.Screen.setFillColor(vex::color(0xFFFFFF));
    Brain.Screen.setPenColor(vex::color(0xc11f27));
    Brain.Screen.printAt(0, 135, "Cibola Robotics");
    this_thread::sleep_for(10);
  }
}
