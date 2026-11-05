using namespace pros;

// Individual motor declarations
// Motor lF(-15);
// Motor lM(16);
// Motor lB(-17);
// Motor rF(18);
// Motor rM(-19);
// Motor rB(20);

// extern Motor LiftMotor;
extern Motor IntakeTop;
extern Motor IntakeBot;
extern MotorGroup Intake;
extern MotorGroup LiftMotors;

// Motor group declarations
// extern MotorGroup Intake;
extern MotorGroup leftMotors;
extern MotorGroup rightMotors;
extern MotorGroup Drivetrain;

// Sensors
extern IMU inertialSensor;
extern Rotation horizontalEnc;
extern Rotation verticalLEnc;
extern Rotation verticalREnc;
extern Rotation LiftRotation;
extern Distance mogoDist;
extern Optical ColorSensor;
extern Optical ParkColor;
// Optical mogoCol(2);

extern Distance distLeft;
extern Distance distRight;
extern pros::Controller controller;
// Const

// Pneumatics (Digital Output)
// #define FrontWingsRightPort 'G'
extern adi::Pneumatics clawLift;
extern adi::Pneumatics clawGrab;
extern adi::Pneumatics doinker;
extern adi::Pneumatics intakeLift; 

