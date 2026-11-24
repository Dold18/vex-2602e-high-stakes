using namespace vex;

#include <array>

extern brain Brain;

struct motorSetup;

extern std::array<motorSetup, 9> motors;

vex::motor makeMotor(std::string name, int32_t port, vex::gearSetting gears, bool reversed);



extern digital_out clampPistons;
extern digital_out dPistons;
extern digital_out raiseIntake;
extern digital_out MMM;

// extern sonar leftSonar;
// extern sonar rightSonar;

extern digital_in ringSwitch;
extern digital_in select;
extern digital_in move;
extern optical colorSensor;
extern distance ringDetector;

extern distance backDist;
extern distance rightDist;
extern distance leftDist;
extern distance frontDist;

extern rotation verticalWheel;
extern rotation ladyBrownTrack;

extern motor LeftFrontMotor;
extern motor LeftMiddleMotor;
extern motor LeftBackMotor;
extern motor RightFrontMotor;
extern motor RightMiddleMotor;
extern motor RightBackMotor;

extern motor_group LeftMotors;
extern motor_group RightMotors;



extern motor LadyBrown;

extern motor_group bMech;


extern motor Intake;
extern motor IntakeWatt5;
extern motor_group IntakeFull;

extern inertial Inertial;
extern analog_in lineDetector;//For auton

// extern motor LeftTest;
// extern motor RightTest;

extern controller Controller1;



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
