#include "botSetup/vex.h"
#include <array>
#include <string>
#include "botSetup/robot-config.h"

using namespace vex;
 
 
//This is the custom motor declarationg used to add motor name and port to a list
struct motorSetup { 
    std::string name;
    int32_t port; 
    vex::motor motorObject = motor(PORT5);//Default    
};  

std::array<motorSetup, 9> motors;

int counter = 0;
vex::motor makeMotor(std::string name, int32_t port, vex::gearSetting gears, bool reversed){
  motors[counter].name = name; 
  motors[counter].port = port;
  motors[counter].motorObject = motor(port, gears, reversed);
  counter++;
  return motors[counter-1].motorObject;
}
 

//SEE MOTOR FUNCS FOR LINE TRACKER PORTS
//SEE MOTOR FUNCS FOR LINE TRACKER PORTS
//SEE MOTOR FUNCS FOR LINE TRACKER PORTS
//SEE MOTOR FUNCS FOR LINE TRACKER PORTS
//SEE MOTOR FUNCS FOR LINE TRACKER PORTS

//port 5 dead

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller Controller1 = controller(primary);

digital_out clampPistons = digital_out(Brain.ThreeWirePort.C);
digital_out dPistons = digital_out(Brain.ThreeWirePort.B);
digital_out raiseIntake = digital_out(Brain.ThreeWirePort.E);
digital_out MMM = digital_out(Brain.ThreeWirePort.H);


digital_in ringSwitch = digital_in(Brain.ThreeWirePort.A);
digital_in select = digital_in(Brain.ThreeWirePort.G);
digital_in move = digital_in(Brain.ThreeWirePort.F);

optical colorSensor = optical(PORT19); 
distance ringDetector = distance(PORT21);

distance backDist = distance(PORT13);
distance rightDist = distance(PORT11);
distance leftDist = distance(PORT10);
distance frontDist = distance(PORT8);


inertial Inertial = inertial(PORT2);
//rotation horizontalWheel = rotation(PORT1, false);
rotation verticalWheel = rotation(PORT5, false);
rotation ladyBrownTrack = rotation(PORT19, true);


motor LeftFrontMotor = makeMotor("LF", PORT3, ratio6_1, false);
//motor LeftFrontMotor = makeMotor("LF", PORT13, ratio6_1, true);
motor LeftMiddleMotor = makeMotor("LM", PORT15, ratio6_1, true);
motor LeftBackMotor = makeMotor("LB", PORT1, ratio6_1, false);
  
motor RightFrontMotor = makeMotor("RF", PORT14, ratio6_1, true);
//motor RightFrontMotor = makeMotor("RF", PORT1, ratio6_1, false);
motor RightMiddleMotor = makeMotor("RM", PORT16, ratio6_1, false);
motor RightBackMotor = makeMotor("RB", PORT18, ratio6_1, true);

motor_group LeftMotors = motor_group(LeftBackMotor, LeftMiddleMotor, LeftFrontMotor);
motor_group RightMotors = motor_group(RightBackMotor, RightMiddleMotor, RightFrontMotor);



motor LadyBrown = makeMotor("LLB", PORT20, ratio18_1, false);

motor_group bMech = motor_group(LadyBrown);


//makeMotor("Intake", PORT15, ratio6_1, true);
motor IntakeWatt5 = motor(PORT9, ratio6_1, true);
motor Intake = motor(PORT12, ratio6_1, false);
motor_group IntakeFull = motor_group(Intake, IntakeWatt5);
 
// motor LeftTest = makeMotor("LF", PORT10, ratio6_1, true);
// motor RightTest = makeMotor("RF", PORT1, ratio6_1, true);
 
//newBrain




void vexcodeInit(void) {
  // Nothing to initialize
}