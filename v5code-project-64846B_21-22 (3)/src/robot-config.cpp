#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Back = motor(PORT5, ratio36_1, false);
motor Claw = motor(PORT1, ratio36_1, true);
motor FrontLeft = motor(PORT13, ratio18_1, false);
motor BackLeft = motor(PORT16, ratio18_1, false);
motor FrontRight = motor(PORT2, ratio18_1, true);
motor BackRight = motor(PORT9, ratio18_1, true);
motor LiftB = motor(PORT3, ratio36_1, false);
motor LiftA = motor(PORT8, ratio36_1, true);
motor_group Lift =  motor_group(LiftA, LiftB);
controller Controller2 = controller(partner);
inertial TurnGyroSmart = inertial(PORT12);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  TurnGyroSmart.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (TurnGyroSmart.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}