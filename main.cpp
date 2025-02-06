#include "main.h"

#define LEFT_MOTOR_A_PORT 1
#define LEFT_MOTOR_B_PORT 4
#define LEFT_MOTOR_C_PORT 7 

#define RIGHT_MOTOR_A_PORT 2
#define RIGHT_MOTOR_B_PORT 6
#define RIGHT_MOTOR_C_PORT 5

#define INERTIAL_PORT 21

#define INTAKE_PORT 3

#define HIGH_STAKES_PORT 8

#define EXT_ADI_SMART_PORT 1
#define CLAMP_PORT 'a'
#define FLAG_PORT 'b'

pros::Motor Intake(-INTAKE_PORT);
pros::Motor HighStakes(HIGH_STAKES_PORT);

pros::MotorGroup LeftDriveSmart({LEFT_MOTOR_A_PORT, LEFT_MOTOR_B_PORT, -LEFT_MOTOR_C_PORT}); //Creates a motor group with forwards ports 1 & 4 and reversed port 7
pros::MotorGroup RightDriveSmart({RIGHT_MOTOR_A_PORT, RIGHT_MOTOR_B_PORT, -RIGHT_MOTOR_C_PORT}); //Creates a motor group with forwards port 2 and reversed port 4 and 7
pros::Imu Inertial(INERTIAL_PORT);
pros::MotorGroup smartdrive ({LEFT_MOTOR_A_PORT, LEFT_MOTOR_B_PORT, -LEFT_MOTOR_C_PORT, RIGHT_MOTOR_A_PORT, RIGHT_MOTOR_B_PORT, -RIGHT_MOTOR_C_PORT, INERTIAL_PORT});
pros::ADIDigitalOut Clamp ({CLAMP_PORT});
pros::ADIDigitalOut Flag ({FLAG_PORT});


bool flagState = false;
bool clampState = false;

void ToggleClamp() {
    clampState = !clampState;          // Toggle the state
    Clamp.set_value(clampState);       // Update the digital output
    pros::delay(200);                // Delay for debouncing
}
void ToggleFlag() {
    flagState = !flagState;          // Toggle the state
    Flag.set_value(flagState);       // Update the digital output
    pros::delay(200);                // Delay for debouncing
}




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	Inertial.reset();
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
void competition_initialize() {}

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



enum Direction {clockwise, counterclockwise};
void TurnDegrees(pros::IMU& inertial, Direction dir, int degrees) {
    Inertial.reset();
    pros::delay(1000);
    int initial = Inertial.get_heading();
    int targetdeg;

    if (dir == clockwise) {
        targetdeg = (initial + degrees) % 360;
        LeftDriveSmart.move_velocity(20);
        RightDriveSmart.move_velocity(20);

        while (inertial.get_heading() < targetdeg) {
            pros::delay(5);
        }
    } else if (dir==counterclockwise) { 
        targetdeg = 360-degrees;
        
        RightDriveSmart.move_velocity(-20);
        LeftDriveSmart.move_velocity(-20);

        while (inertial.get_heading() > targetdeg || inertial.get_heading() < 5) {
            pros::delay(5);
        }
    }

    // Stop the motors
    LeftDriveSmart.move_velocity(0);
    RightDriveSmart.move_velocity(0);
}

void moveForward(int speed){
    RightDriveSmart.move_velocity(-speed);
    LeftDriveSmart.move_velocity(speed);
}
void driveStop(){
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
}
void turn(int speed, int dir){
    if (dir = 1) {
    RightDriveSmart.move_velocity(-speed);
    LeftDriveSmart.move_velocity(-speed);
    // turn left
    }
    if (dir = 0) {
    RightDriveSmart.move_velocity(speed);
    LeftDriveSmart.move_velocity(speed);
    // turn right
    }
}

void autonomous() {
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    
    // negitive side auton
    //----------------------------------------------------------------
    //TurnDegrees(Inertial, clockwise/counterclockwise, degrees);
    // RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //Inertial.reset();
    //pros::delay(2200);

    // ToggleClamp(); //the clamp starts at true then moves to false
    // pros::delay(500);

    // HighStakes.move_relative(-600, 100);
    
    // moveForward(80);
    // pros::delay(1500);
    // ToggleClamp();// grabs the moble goal, sets clamp to true
    // driveStop();

    // Intake.move_velocity(200);  
    // pros::delay(1200);
    // Intake.move_velocity(0); // scores preload
    // pros::delay(100);
    // turn(80, 0);
    // pros::delay(300);
    // driveStop();
    // moveForward(-80);
    // Intake.move_velocity(200);
    // pros::delay(650);
    // driveStop();
    // pros::delay(1500); // scores 1st ring
    // Intake.move_velocity(0);
    // moveForward(-80);
    // pros::delay(75);
    // driveStop();
    // turn(80, 0);
    // pros::delay(400);
    // driveStop();
    // moveForward(-80);
    // Intake.move_velocity(200);
    // pros::delay(550);
    // driveStop();
    // pros::delay(2000);
    // Intake.move_velocity(0); // scores 2nd ring
    // turn(-80, 0); this is commented out because it will cross the line
    // pros::delay(225);
    // driveStop();
    // pros::delay(100);
    // moveForward(-80);
    // Intake.move_velocity(200);
    // pros::delay(250);
    // driveStop();
    // pros::delay(2000);
    // Intake.move_velocity(0);
    // Auton for skills
    //---------------------------------------------
    /*RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Inertial.reset();
    pros::delay(2000);
    ToggleClamp();// clamp starts at false so moves to true
    HighStakes.move_relative(-600,100);// moves high stake up in order for it to be out of the way for the intake
    pros::delay(500);
    LeftDriveSmart.move_velocity(70);// drives in reverse
    RightDriveSmart.move_velocity(-70);
    pros::delay(800);
    ToggleClamp();// sets the clamp from true to false
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    Intake.move_velocity(200);// scores the ring into the moble goal
    pros::delay(700);
    Intake.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);// turns counterclockwise
    LeftDriveSmart.move_velocity(80);
    pros::delay(680);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);// drives forwards
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);// activates intake in order to pick up ring
    pros::delay(700);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(1500);
    Intake.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);// turns counterclockwise
    LeftDriveSmart.move_velocity(80);
    pros::delay(495);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);// drives forwards
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);// activates intake in order to pick up ring
    pros::delay(900);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(1500);
    Intake.move_velocity(0);
    
    pros::delay(100);

    RightDriveSmart.move_velocity(80);// turns counterclockwise
    LeftDriveSmart.move_velocity(80);
    pros::delay(495);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);// drives forwards
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);// activates intake in order to pick up ring
    pros::delay(1300);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(-80);// turns clockwise
    LeftDriveSmart.move_velocity(-80);
    pros::delay(680);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);// drives forwards
    LeftDriveSmart.move_velocity(-80);
    pros::delay(500);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(1500);
    Intake.move_velocity(0);

    pros::delay(100);

    TurnDegrees(Inertial, clockwise, 77);

    pros::delay(100);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(300);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    ToggleClamp();

    pros::delay(100);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(300);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    TurnDegrees(Inertial, counterclockwise, 85);*/
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
    pros::Controller Controller1(pros::E_CONTROLLER_MASTER);

    while (true) {
        // Calculate drivetrain motor velocities
        // Left joystick (up/down) for forward/backward (Axis3)
        // Right joystick (left/right) for turning (Axis1)
        int turn = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Forward/backward
        int forward = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Turning

        // Compute motor speeds for tank drive
        int drivetrainLeftSideSpeed = (forward - turn);  // Left motor speed
        int drivetrainRightSideSpeed = -(forward + turn); // Right motor speed

        // Deadband logic to prevent small joystick movements from moving the robot
        const int deadband = 25; // Threshold for joystick input
        if (abs(drivetrainLeftSideSpeed) < deadband) {
            drivetrainLeftSideSpeed = 0;
        }
        if (abs(drivetrainRightSideSpeed) < deadband) {
            drivetrainRightSideSpeed = 0;
        }

        // Set motor velocities
        LeftDriveSmart.move_velocity((drivetrainRightSideSpeed * 2));  // Adjust scaling as needed
        RightDriveSmart.move_velocity(-(drivetrainLeftSideSpeed * 2));

        // Control Clamp and Flag using buttons
        if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            ToggleClamp();
        }
        if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            ToggleFlag();
        }

        // Control Intake using shoulder buttons (L1/L2)
        if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            Intake.move_velocity(200);  // Spin intake forward
        } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            Intake.move_velocity(-200); // Spin intake backward
        } else {
            Intake.move_velocity(0);    // Stop intake
        }
        if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            HighStakes.move_velocity(-100);  // Spin intake forward
        } else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            HighStakes.move_velocity(100); // Spin intake backward
        } else {
            HighStakes.move_velocity(0);    // Stop intake
        }

        // Delay to prevent CPU overload
        pros::delay(20);
    }
}
