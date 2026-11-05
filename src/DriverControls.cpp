#include "main.h"
using namespace pros;


int drivecontrol() {
  // Check if either joystick is moved beyond the deadzone threshold (±1)
  if (abs(controller.get_analog(ANALOG_LEFT_Y)) > 1 || abs(controller.get_analog(ANALOG_RIGHT_X)) > 1) { 
    // Read the vertical axis of the left joystick (forward/reverse motion)
    int fwdrev = controller.get_analog(ANALOG_LEFT_Y); 
    
    // Read the horizontal axis of the right joystick (turning), scaled by 1.2 for sharper turns
    int riglft = controller.get_analog(ANALOG_RIGHT_X) * 0.85; 

    // Calculate speed for left motors by combining forward/reverse and turning inputs
    int lSpeed = fwdrev + riglft; 
    
    // Calculate speed for right motors by subtracting turning input from forward/reverse
    int rSpeed = fwdrev - riglft; 

    // Command the motors to spin at the calculated speed
    leftMotors.move(lSpeed); 
    rightMotors.move(rSpeed); 
  } 
  else {  // If joysticks are within the deadzone, stop the drivetrain
    // Set brake mode for both motors
    rightMotors.set_brake_mode_all(E_MOTOR_BRAKE_BRAKE); 
    leftMotors.set_brake_mode_all(E_MOTOR_BRAKE_BRAKE); 

    // Apply brakes to motors
    rightMotors.brake(); 
    leftMotors.brake(); 
  }
  return 0;
}



// bool LiftToggle = 0;
// bool MatchLoadToggle = 0;
// bool BunnyToggle = 1;
// bool doubleParkToggle = 0;


// int pistonControl(){ 
//   if (controller.get_digital(DIGITAL_B)){ 
//   //detects when Button Right 2 is pressed
//     LiftToggle = !LiftToggle; 
//     Lift.set_value(LiftToggle);
//     delay(300);
//   }
//   Lift.set_value(LiftToggle);
//   if (controller.get_digital(DIGITAL_Y)){ 
//     //detects when Button Right 2 is pressed
//       MatchLoadToggle = !MatchLoadToggle; 
//       matchLoader.set_value(MatchLoadToggle);
//       descore.set_value(!MatchLoadToggle);
//       delay(300);
//     }
//   matchLoader.set_value(MatchLoadToggle);
//   descore.set_value(!MatchLoadToggle);
  
//    if (controller.get_digital(DIGITAL_RIGHT)){ 
//     //detects when Button Right 2 is pressed
//       doubleParkToggle = !doubleParkToggle; 
//       DoublePark.set_value(doubleParkToggle);
//       delay(300);
//     }
//   DoublePark.set_value(doubleParkToggle);

//   if (controller.get_digital(DIGITAL_DOWN)){ 
//     //detects when Button Right 2 is pressed
//       BunnyToggle = !BunnyToggle; 
//       BunnyDescore.set_value(BunnyToggle);
//       delay(300);
//     }
//   BunnyDescore.set_value(BunnyToggle);
//   return 0;
// }


int intakeControl(){
  if (controller.get_digital(DIGITAL_L1)){ 
    // Detects when Button Left 1 is being pressed
    Intake.move(100); 
    intakeLift.set_value(false);
    // spins the intake forward when the if statement above is true
  }
  else if (controller.get_digital(DIGITAL_L2)){
    // Detects when Button Left 2 is being pressed
    Intake.move(-127); 
    intakeLift.set_value(false);
    // spins the intake backward when the if statement above is true
  }
  else if(controller.get_digital(DIGITAL_A)){
    // Detects when Button Left 2 is being pressed
    Intake.move(30); 
    intakeLift.set_value(true);
    // spins the intake backward when the if statement above is true
  }
  else {
    Intake.brake(); 
  }
  return 0;
}

 bool clawToggle = 0;
 bool scoring = 0;

  // Make a macro to get ready for scoring
  // Make a macro for coming back down


int scoringControl(){
  // if (controller.get_digital(DIGITAL_B)){ 
  //   //detects when Button Right 2 is pressed
  //     clawToggle = !clawToggle; 
  //     clawGrab.set_value(clawToggle);
  //     delay(300);
  //   }
  clawGrab.set_value(clawToggle);


  if (controller.get_digital(DIGITAL_R1)){ 
    // Detects when Button Left 1 is being pressed
    LiftMotors.move(127); 
    clawLift.set_value(true);
    if (scoring == 0){
      clawToggle = 1;
    }
    // spins the lift motors forward when the if statement above is true
  }
  else if (controller.get_digital(DIGITAL_R2)){
    // Detects when Button Left 2 is being pressed
    LiftMotors.move(-90); 
    // spins the lift motors backward when the if statement above is true
  }
  else if (controller.get_digital(DIGITAL_B)){
    // Detects when Button Left 2 is being pressed
    dropLiftMacro();
    scoring = 0;
    // spins the lift motors backward when the if statement above is true
  }
  else if (controller.get_digital(DIGITAL_Y)){
    scoreMacro();
    scoring = 1;
  }
  else {
    LiftMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    LiftMotors.brake(); 
  }
  return 0;
}

bool doinkerToggle = 0;

int doinkerControl(){
  if (controller.get_digital(DIGITAL_UP)){ 
    doinkerToggle = !doinkerToggle;
    doinker.set_value(doinkerToggle); 
    delay(200);
  }

  doinker.set_value(doinkerToggle);
  return 0;
}

