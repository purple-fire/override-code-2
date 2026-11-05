#include "main.h"
#include "lemlib/api.hpp"
#include <string>


lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, (0));
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel1(&verticalREnc, lemlib::Omniwheel::NEW_2, (3.125));
lemlib::TrackingWheel vertical_tracking_wheel2(&verticalLEnc, lemlib::Omniwheel::NEW_2, (0));


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.375, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              3.7 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1 &horizontal_tracking_wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertialSensor // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(11, // proportional gain (kP)
                                              0, // integral gain (kI) .06
                                              50, // derivative gain (kD)
                                              0.05,// anti windup
                                              .01, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              .02, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12.5, // derivative gain (kD)
                                              3, // anti windup
                                              .01, // small error range, in degrees
                                              1, // small error range timeout, in milliseconds
                                              .25, // large error range, in degrees
                                              20, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

float LeftXFC = 2.5;
float LeftYFC = 0;
float RightXFC = 2.5;
float RightYFC = 0;

double DeltaY = 0;
double DeltaX = 0;
double CurrentY = 0;
double CurrentX = 0;
float Theta = 0;
float CornerX = 70.5;
float CornerY = 70.5;
float FinalX = 0;
float FinalY = 0;


void POSReset(int Quadrant){	
	// quadrantCheck();
	double Left = (distLeft.get_distance() / 25.4) + LeftXFC;
	double Right = (distRight.get_distance() / 25.4) + RightXFC;

	if (Quadrant == 1){
		Theta = abs(keepInRange360(chassis.getPose().theta));
		DeltaY = Left*cos((90 - Theta)*(3.14/180)) + (RightYFC * sin((90 - Theta)*(3.14/180)));
		DeltaX = Right*cos((Theta)*(3.14/180)) + (LeftYFC * cos((90 - Theta)*(3.14/180)));
		FinalX = (CornerX - DeltaX);
		FinalY = (CornerY - DeltaY);
	}
	else if (Quadrant == 2){
		Theta = abs(keepInRange360(chassis.getPose().theta) - 270);
		DeltaY = Right*cos((Theta)*(3.14/180)) + (RightYFC * sin((Theta)*(3.14/180)));
		DeltaX = Left*cos((90 - Theta)*(3.14/180)) + (LeftYFC * cos((Theta)*(3.14/180)));
		FinalX = -1*(CornerX - DeltaX);
		FinalY = (CornerY - DeltaY);
	}
	else if (Quadrant == 3){
		Theta = abs(270 - keepInRange360(chassis.getPose().theta));
		DeltaY = Left*cos((Theta)*(3.14/180)) + (RightYFC * sin((Theta)*(3.14/180)));
		DeltaX = Right*cos((90 - Theta)*(3.14/180)) + (LeftYFC * cos((Theta)*(3.14/180)));
		FinalX = -1*(CornerX - DeltaX);
		FinalY = -1*(CornerY - DeltaY);
	}
	else if (Quadrant == 4){
		Theta = abs(keepInRange360(chassis.getPose().theta) - 90);
		DeltaY = Right*cos((Theta)*(3.14/180)) + (RightYFC * sin((Theta)*(3.14/180)));
		DeltaX = Left*cos((90 - Theta)*(3.14/180)) + (LeftYFC * cos((Theta)*(3.14/180)));
		FinalX = (CornerX - DeltaX);
		FinalY = -1*(CornerY - DeltaY);
	}

	chassis.setPose(FinalX, FinalY, keepInRange360(chassis.getPose().theta));

}

std::pair<float, int> closestRefAng(bool axis){
	float referenceAngle;
	int specificAng;
	float currentAng = keepInRange360(chassis.getPose().theta);
	if (axis == 1) {
		if (currentAng < 90){
			referenceAngle = 90 - (90 - currentAng);
			specificAng = 1;
		}
		else if (currentAng > 90 && currentAng < 180){
			referenceAngle = (currentAng - 90);
			specificAng = 1;
		}
		else if (currentAng > 180 && currentAng < 270){
			referenceAngle = (270 - currentAng);
			specificAng = 0;
		}
		else{
			referenceAngle = 90 - (currentAng - 270);
			specificAng = 0;
		}
	}
	else if (axis == 0) {
		if (currentAng < 90){
			referenceAngle = (90 - currentAng);
			specificAng = 1;
		}
		else if (currentAng > 90 && currentAng < 180){
			referenceAngle = (currentAng - 90);
			specificAng = 0;
		}
		else if (currentAng > 180 && currentAng < 270){
			referenceAngle = (270 - currentAng);
			specificAng = 0;
		}
		else{
			referenceAngle = (currentAng - 270);
			specificAng = 1;
		}
	}
	return {referenceAngle, specificAng};

}

// The first parameter 1 is L and 0 is R
// Second parameter 1 is X and 0 is Y
// Third is if the coordinate should be negative or not
float singleCoordReset(bool LR, bool XY, int neg, bool reset){
	float sensor;
	if (LR == 1){
		sensor = distLeft.get_distance();
	}
	else if (LR == 0){
		sensor = distRight.get_distance();
	}
	float calCoord;
		if (closestRefAng(XY).second == 1){
			calCoord = 70.5 - (cos(fabs(closestRefAng(XY).first * (3.14/180))) * ((sensor / 25.4) + RightXFC)) - (RightYFC * sin(closestRefAng(XY).first * (3.14/180)));
		}
		else {
			calCoord = 70.5 - (cos(fabs(closestRefAng(XY).first * (3.14/180))) * ((sensor / 25.4) + RightXFC)) + (RightYFC * sin(closestRefAng(XY).first * (3.14/180)));
		}
	//________________________________________________________________________________________________________________________
	if (reset == true){
		if (XY == 0){
			chassis.setPose(chassis.getPose().x, neg * calCoord, chassis.getPose().theta);
		}
		else if (XY == 1){
			chassis.setPose(neg * calCoord, chassis.getPose().y, chassis.getPose().theta);
		}
	}
	printf("Dist: %d\n", sensor);
	return (calCoord);
			// pros::lcd::print(0, "calCoord: %f", calCoord); // x
            // pros::lcd::print(1, "sensor: %f", sensor / 25.4 + RightXFC); // y
            // pros::lcd::print(2, "closestRefAng: %f", closestRefAng(XY).first);
			// pros::lcd::print(3, "Theta: %f", keepInRange360(chassis.getPose().theta)); // heading
			// pros::lcd::print(4, "Left: %f", chassis.getPose().x); // x
            // pros::lcd::print(5, "extra: %f", (RightYFC * sin(closestRefAng(XY).first * (3.14/180)))); // y
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// this runs at the start of the program
// LV_IMG_DECLARE(OC);

void initialize() { 

	// lv_obj_t * img = lv_img_create(lv_scr_act());
	// lv_img_set_src(img, &OC);
	// lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
	LiftMotors.set_encoder_units_all(E_MOTOR_ENCODER_DEGREES);
	LiftMotors.tare_position_all();
	inertialSensor.tare();

	chassis.setPose(0, 0, 0);
	// pros::Task screen_task([&]() {
		
    //     while (true) {
	// 		 pros::lcd::print(1, "x: %f", chassis.getPose().x);
	// 		 pros::lcd::print(2, "y: %f", chassis.getPose().y);
	// 		 pros::lcd::print(3, "theta: %f", chassis.getPose().theta);
	// 		//   pros::lcd::print(4, "leftPOS: %f", leftMotors.get_position());
	// 		//   pros::lcd::print(5, "rightPOS: %f", rightMotors.get_position());
	// 		//   pros::lcd::print(6, "encoder: %f", horizontalEnc.get_position());
	// 		//   singleCoordReset(1, 0, 1);
    //         pros::delay(20);
    //     }
    // });

	
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	
}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
	// BunnyDescore.set_value(true);
	pros::Task screen_task([&]() {
		
        while (true) {
			 pros::lcd::print(1, "x: %f", chassis.getPose().x);
			 pros::lcd::print(2, "y: %f", chassis.getPose().y);
			 pros::lcd::print(3, "theta: %f", chassis.getPose().theta);
			//   pros::lcd::print(4, "leftPOS: %f", leftMotors.get_position());
			//   pros::lcd::print(5, "rightPOS: %f", rightMotors.get_position());
			//   pros::lcd::print(6, "encoder: %f", horizontalEnc.get_position());
			//   singleCoordReset(1, 0, 1);
            pros::delay(20);
        }
    });
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 48, 4000, {.forwards = true, .maxSpeed = 70});

	chassis.turnToHeading(90, 4000, {.maxSpeed = 70});
	

	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	// pros::Task print_task([&]() {
		
    //     while (true) {
	// 		 pros::lcd::print(1, "pos: %f", (LiftMotors.get_position(0) + LiftMotors.get_position(1)) / 2);
    //         pros::delay(20);
    //     }
    // });

controller.clear();
	pros::Task driveTask([&]() { 
	    while (true){
		    drivecontrol();
			delay(10);   
			}
		}); 
	pros::Task otherTask([&]() { 
		while (true){
			intakeControl();
			doinkerControl();
			delay(10);
		}
	}); 
	pros::Task scoreTask([&]() { 
		while (true){
			scoringControl();
			delay(10);
		}
	}); 

}

