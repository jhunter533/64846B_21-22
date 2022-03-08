using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Claw;
extern motor Back;
extern motor FrontLeft;
extern motor BackLeft;
extern motor FrontRight;
extern motor BackRight;
extern motor_group Lift;
extern controller Controller2;
extern inertial TurnGyroSmart;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );