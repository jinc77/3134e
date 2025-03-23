#include "main.h"
#include "pros/motors.hpp"
#include <cmath>

#define LEFT_MOTOR_A_PORT 18
#define LEFT_MOTOR_B_PORT 20
#define LEFT_MOTOR_C_PORT 16

#define RIGHT_MOTOR_A_PORT 14
#define RIGHT_MOTOR_B_PORT 13
#define RIGHT_MOTOR_C_PORT 11

#define INERTIAL_PORT 21

#define INTAKE_PORT 3
#define ROTATION_PORT 1
#define HIGH_STAKES_PORT 6

#define EXT_ADI_SMART_PORT 1
#define CLAMP_PORT1 'a'
#define CLAMP_PORT2 'c'
#define FLAG_PORT 'b'

#define SCORE_ANGLE 208.30
#define PICKUP_ANGLE 357.78

pros::Motor Intake(-INTAKE_PORT);

pros::Rotation RotationSensor(ROTATION_PORT);

pros::Motor HighStakes(HIGH_STAKES_PORT);
// pros::Motor HighStakes(port, pros::motor_gearset_e:: E_MOTOR_GEAR_RED);

pros::MotorGroup LeftDriveSmart({LEFT_MOTOR_A_PORT, LEFT_MOTOR_B_PORT, LEFT_MOTOR_C_PORT});     // Creates a motor group with forwards ports 1 & 4 and reversed port 7
pros::MotorGroup RightDriveSmart({RIGHT_MOTOR_A_PORT, RIGHT_MOTOR_B_PORT, RIGHT_MOTOR_C_PORT}); // Creates a motor group with forwards port 2 and reversed port 4 and 7
pros::Imu Inertial(INERTIAL_PORT);
pros::MotorGroup smartdrive({LEFT_MOTOR_A_PORT, LEFT_MOTOR_B_PORT, -LEFT_MOTOR_C_PORT, RIGHT_MOTOR_A_PORT, RIGHT_MOTOR_B_PORT, -RIGHT_MOTOR_C_PORT, INERTIAL_PORT});
pros::ADIDigitalOut Clamp1({CLAMP_PORT1});
pros::ADIDigitalOut Clamp2({CLAMP_PORT2});
pros::ADIDigitalOut Flag({FLAG_PORT});

bool flagState = false;
bool clampState = false;

void Skills();
void Red_Negative();
void Blue_Negative();
void Blue_Positive();
void Red_Positive();

void ToggleClamp()
{
    clampState = !clampState;     // Toggle the state
    Clamp1.set_value(clampState); // Update the digital output
    Clamp2.set_value(clampState); // Update the digital output
    pros::delay(200);             // Delay for debouncing
}
void ToggleFlag()
{
    flagState = !flagState;    // Toggle the state
    Flag.set_value(flagState); // Update the digital output
    pros::delay(200);          // Delay for debouncing
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
    static bool pressed = false;
    pressed = !pressed;
    if (pressed)
    {
        pros::lcd::set_text(2, "I was pressed!");
    }
    else
    {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    pros::lcd::register_btn1_cb(on_center_button);
    Inertial.reset();
    RotationSensor.reset();
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

void scoreHighStakes()
{
    while ((RotationSensor.get_angle() / 100) < SCORE_ANGLE)
    {
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Angle Is less than 360 : %3d", RotationSensor.get_angle() / 100);
        HighStakes.move_velocity(-100);
    }
    HighStakes.move_velocity(0);

    while ((RotationSensor.get_angle() / 100) >= SCORE_ANGLE)
    { // could do < or <=
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Angle is greater than 360: %3d", RotationSensor.get_angle() / 100);
        HighStakes.move_velocity(100);
    }
    HighStakes.move_velocity(0);

    return;
}

void pickupHighStakes()
{

    while ((RotationSensor.get_angle() / 100) < PICKUP_ANGLE)
    {
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Angle Is less than 360 : %3d", RotationSensor.get_angle() / 100);
        HighStakes.move_velocity(-100);
    }
    HighStakes.move_velocity(0);

    while ((RotationSensor.get_angle() / 100) >= PICKUP_ANGLE)
    { // could do < or <=
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Angle is greater than 360: %3d", RotationSensor.get_angle() / 100);
        HighStakes.move_velocity(100);
    }
    HighStakes.move_velocity(0);

    return;
}

enum Direction
{
    clockwise,
    counterclockwise
};

enum forback
{
    forward,
    backward
};

void TurnDegrees(pros::IMU &inertial, Direction dir, int degrees)
{
    Inertial.reset();
    pros::delay(1000);
    int initial = Inertial.get_heading();
    int targetdeg;

    if (dir == clockwise)
    {
        targetdeg = (initial + degrees) % 360;
        LeftDriveSmart.move_velocity(20);
        RightDriveSmart.move_velocity(20);

        while (inertial.get_heading() < targetdeg)
        {
            pros::delay(5);
        }
    }
    else if (dir == counterclockwise)
    {
        targetdeg = 360 - degrees;

        RightDriveSmart.move_velocity(-20);
        LeftDriveSmart.move_velocity(-20);

        while (inertial.get_heading() > targetdeg || inertial.get_heading() < 5)
        {
            pros::delay(5);
        }
    }

    // Stop the motors
    LeftDriveSmart.move_velocity(0);
    RightDriveSmart.move_velocity(0);
}

void inertialMove(int speed, int duration, forback FB)
{
    // DURATION IN MILLISECONDS

    int initial = Inertial.get_heading(); // Get initial heading
    
    int leftSpeed;
    int rightSpeed;

    if (FB == backward)
    {
        rightSpeed = -speed;
        leftSpeed = speed;
    }
    else
    {
        leftSpeed = -speed;
        rightSpeed = speed;
    }
    double kp = .1;
    int endTime = pros::millis() + duration;

    while (pros::millis() < endTime)
    {
        int currentHeading = Inertial.get_heading();
        int error = currentHeading - initial;
        int correction = error * kp;

        // Apply correction temporarily without modifying base speeds
        LeftDriveSmart.move_velocity(leftSpeed + correction);
        RightDriveSmart.move_velocity(rightSpeed - correction);

        pros::delay(5); // Small delay for loop efficiency
    }

    // Stop the motors after duration ends
    LeftDriveSmart.move_velocity(0);
    RightDriveSmart.move_velocity(0);
}

void moveForward(int speed)
{
    RightDriveSmart.move_velocity(-speed);
    LeftDriveSmart.move_velocity(speed);
}
void driveStop()
{
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
}
void turn(int speed, int dir)
{
    if (dir = 1)
    {
        RightDriveSmart.move_velocity(-speed);
        LeftDriveSmart.move_velocity(-speed);
        // turn left
    }
    if (dir = 0)
    {
        RightDriveSmart.move_velocity(speed);
        LeftDriveSmart.move_velocity(speed);
        // turn right
    }
}

void autonomous()
{
    // Skills();
    // Red_Negative();
    // Blue_Negative();
    // Blue_Positive();
    // Red_Positive();
}
void Blue_Positive()
{
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ToggleClamp();

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(1250);
    ToggleClamp();
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);
    Intake.move_velocity(200);
    pros::delay(2000);
    Intake.move_velocity(0);
    ToggleClamp();

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(335);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(900);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(350);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(400);
    ToggleFlag();
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);
    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(500);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);
    ToggleFlag(); // takes off the mobel goal (next is to camp it)

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(300);
    ToggleClamp();
    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(200);

    Intake.move_velocity(200);
    pros::delay(2000);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
}

void Red_Positive()
{
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ToggleClamp();

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(1250);
    ToggleClamp();
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);
    Intake.move_velocity(200);
    pros::delay(2000);
    Intake.move_velocity(0);
    ToggleClamp();

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(335);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(900);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(350);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(400);
    ToggleFlag();
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);
    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(500);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(100);
    ToggleFlag(); // takes off the mobel goal (next is to camp it)

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(300);
    ToggleClamp();
    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(200);

    Intake.move_velocity(200);
    pros::delay(2000);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
}

void Blue_Negative()
{
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    ToggleClamp(); // the clamp starts at true then moves to false
    pros::delay(500);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(1250);
    ToggleClamp(); // grabs the moble goal, sets clamp to true
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    Intake.move_velocity(200);
    pros::delay(1200);
    Intake.move_velocity(0); // scores preload

    pros::delay(100);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(220);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(500);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(1800);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(190);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(1000);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(100);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(300);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(2000);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(-90);
    LeftDriveSmart.move_velocity(80);
    pros::delay(2000);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(300);
    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
}

void Skills()
{
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //Intake.move_velocity(0);
    ToggleClamp();                        // clamp starts at false so moves to true
    pros::delay(500);

    LeftDriveSmart.move_velocity(70); // drives in reverse
    RightDriveSmart.move_velocity(-70);
    pros::delay(750);
    ToggleClamp(); // sets the clamp from true to false
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    Intake.move_velocity(200); // scores the preload into the moble goal
    pros::delay(1000);
    Intake.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80); // turns counterclockwise
    LeftDriveSmart.move_velocity(80);
    pros::delay(550);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    pros::delay(100);

    RightDriveSmart.move_velocity(80); // drives forwards
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200); // activates intake in order to pick up ring
    pros::delay(650);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(2000);
    Intake.move_velocity(0);

    pros::delay(100);

    // RightDriveSmart.move_velocity(80); // turns counterclockwise
    // LeftDriveSmart.move_velocity(80);
    // pros::delay(570);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80); // drives forwards
    // LeftDriveSmart.move_velocity(-80);
    // Intake.move_velocity(200); // activates intake in order to pick up ring
    // pros::delay(880);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);
    // pros::delay(2000);
    // Intake.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80); // turns counterclockwise
    // LeftDriveSmart.move_velocity(80);
    // pros::delay(450);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80); // drives forwards
    // LeftDriveSmart.move_velocity(-80);
    // Intake.move_velocity(200); // activates intake in order to pick up ring
    // pros::delay(1300);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(600);

    // RightDriveSmart.move_velocity(-80); // turns clockwise
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(600);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80); // drives forwards
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(450);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);
    // pros::delay(2000);
    // Intake.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(-80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(600);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(-80);
    // LeftDriveSmart.move_velocity(80);
    // pros::delay(250);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);
    // ToggleClamp();

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(280);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // RightDriveSmart.move_velocity(80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(1050);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80);
    // LeftDriveSmart.move_velocity(80);
    // pros::delay(270);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(2000);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(-80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(250);
    // LeftDriveSmart.move_velocity(0);
    // RightDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(2050);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(-80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(340);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(-80);
    // LeftDriveSmart.move_velocity(80);
    // pros::delay(1800);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);

    // pros::delay(100);

    // RightDriveSmart.move_velocity(80);
    // LeftDriveSmart.move_velocity(-80);
    // pros::delay(4700);
    // RightDriveSmart.move_velocity(0);
    // LeftDriveSmart.move_velocity(0);
}

void negative()
{
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    HighStakes.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    ToggleClamp(); // the clamp starts at true then moves to false
    pros::delay(500);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(1250);
    ToggleClamp(); // grabs the moble goal, sets clamp to true
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    Intake.move_velocity(200);
    pros::delay(1200);
    Intake.move_velocity(0); // scores preload

    pros::delay(100);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(220);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(500);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(1800);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(-80);
    pros::delay(190);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(400);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(1000);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(80);
    pros::delay(100);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(300);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);
    pros::delay(2000);
    Intake.move_velocity(0);

    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(90);
    pros::delay(2000);
    RightDriveSmart.move_velocity(0);
    LeftDriveSmart.move_velocity(0);

    RightDriveSmart.move_velocity(80);
    LeftDriveSmart.move_velocity(-80);
    Intake.move_velocity(200);
    pros::delay(300);
    RightDriveSmart.move_velocity(-80);
    LeftDriveSmart.move_velocity(80);

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

void opcontrol()
{
    pros::Controller Controller1(pros::E_CONTROLLER_MASTER);

    while (true)
    {
        // Calculate drivetrain motor velocities
        // Left joystick (up/down) for forward/backward (Axis3)
        // Right joystick (left/right) for turning (Axis1)
        int turn = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);     // Forward/backward
        int forward = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // Turning

        // Compute motor speeds for tank drive
        int drivetrainLeftSideSpeed = (forward - turn);   // Left motor speed
        int drivetrainRightSideSpeed = -(forward + turn); // Right motor speed

        // Deadband logic to prevent small joystick movements from moving the robot
        const int deadband = 25; // Threshold for joystick input
        if (abs(drivetrainLeftSideSpeed) < deadband)
        {
            drivetrainLeftSideSpeed = 0;
        }
        if (abs(drivetrainRightSideSpeed) < deadband)
        {
            drivetrainRightSideSpeed = 0;
        }

        // Set motor velocities
        LeftDriveSmart.move_velocity((drivetrainRightSideSpeed * 2)); // Adjust scaling as needed
        RightDriveSmart.move_velocity(-(drivetrainLeftSideSpeed * 2));

        // Control Clamp and Flag using buttons
        if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            ToggleClamp();
        }
        if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            ToggleFlag();
        }
        if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            scoreHighStakes();
        }
        if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            HighStakes.move_velocity(100);
            pros::delay(200);
            HighStakes.move_velocity(0);
        }

        // Control Intake using shoulder buttons (L1/L2)
        if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            Intake.move_velocity(200); // Spin intake forward
        }
        else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            Intake.move_velocity(-200); // Spin intake backward
        }
        else
        {
            Intake.move_velocity(0); // Stop intake
        }

        if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            HighStakes.move_velocity(-100); // Spin high stakes forward
        }
        else if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            HighStakes.move_velocity(100); // Spin high stakes backward
        }
        else
        {
            HighStakes.move_velocity(0); // Stop high stakes
        }

        // Delay to prevent CPU overload
        pros::delay(20);
    }
}
