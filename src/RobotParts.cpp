#include "main.h"
using namespace pros;

// Individual motor declarations
// Motor lF(-15);
// Motor lM(16);
// Motor lB(-17);
// Motor rF(18);
// Motor rM(-19);
// Motor rB(20);

// Motor LiftMotor(18);
// Motor Intake1(6);

// Motor group declarations
MotorGroup Intake({-9, 10}); 
MotorGroup LiftMotors({-11, 17});

MotorGroup leftMotors({-4, 3, -1, 2});
MotorGroup rightMotors({6, -8, 7, -5});
MotorGroup Drivetrain({-1, 2, 3, -4, -5, 6, 7, -8});

// Sensors
IMU inertialSensor(12);
Rotation horizontalEnc(19);
Rotation verticalLEnc(100);
Rotation verticalREnc(100);
Rotation LiftRotation(100);
Distance mogoDist(100);
Optical ColorSensor(17);
Optical ParkColor(100);
// Optical mogoCol(2);

Distance distLeft(14);

Distance distRight(15);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Const

// Pneumatics
adi::Pneumatics clawLift('G', false); //Lift
adi::Pneumatics doinker('C', false); // DOINKNER
adi::Pneumatics clawGrab('H', false); 
adi::Pneumatics intakeLift('B', false); 