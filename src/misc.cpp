#include "main.h"

void decelerateScoring(int initialSpeed, int decelRate, int time){
    IntakeBot.move(-127);
    IntakeTop.move(-127);
    delay(150);
    int speed = initialSpeed;
    IntakeBot.move(speed);
    IntakeTop.move(speed);
    delay(time - 500);
    while (speed > 40){
        IntakeBot.move(speed);
        IntakeTop.move(speed);
        delay(100);
        speed -= decelRate;
    }
        IntakeBot.move(speed + 15);
        IntakeTop.move(speed + 15);
        delay(500);
        IntakeBot.brake();
        IntakeTop.brake();
}


float keepInRange360(float num){
  while(num>=360 || num< 0){
    if(num<0){
      num += 360;
    }else{
      num -=360;
    }
  }
  return num;
}

float keepInNeg180to180(float num){
  while(num>=180 || num<-180){
    if(num>=180){
      num-= 360;
    }else{
      num+=360;
    }
  }
  return num;
}

void scoreMacro(){
  clawToggle = 0;
  clawGrab.set_value(false);
  LiftMotors.move(-100);
}


void dropLiftMacro(){
  LiftMotors.set_encoder_units_all(E_MOTOR_ENCODER_DEGREES);
  int liftPosition = (LiftMotors.get_position(0) + LiftMotors.get_position(1)) / 2;
  int speed;
  int timeout = 0;

  clawToggle = 1;
  clawGrab.set_value(clawToggle);
  clawLift.set_value(false);

  while (abs(liftPosition) > 30){
    liftPosition = (LiftMotors.get_position(0) + LiftMotors.get_position(1)) / 2;
    speed = std::clamp<int>(-liftPosition / 2, -100, 50);
    LiftMotors.move(speed);

    timeout += 20;
    if (timeout > 3000){
      break;
    }
    delay(20);
  }

  clawToggle = 0;
  clawGrab.set_value(clawToggle);
}

void liftToHeight(void* param){
  int targetPos = *static_cast<int*>(param);
  int currentPos = (LiftMotors.get_position(0) + LiftMotors.get_position(1)) / 2;
  int speed;

  while (abs(targetPos - currentPos) > 30){
    currentPos = (LiftMotors.get_position(0) + LiftMotors.get_position(1)) / 2;
    speed = std::clamp<int>((targetPos - currentPos) / 4, -50, 127);
    LiftMotors.move(speed);
  }
  LiftMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
  LiftMotors.brake();
}

void lift_async(int POS, bool async){
  static int positionParam = POS;

  if (async){
    pros::Task liftTask(liftToHeight, &positionParam, "Lift Task");
  } else {
    liftToHeight(&positionParam);
  }
}