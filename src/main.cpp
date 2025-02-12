#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

ASSET(first_pylon_1_txt);
ASSET(five_ring_2_txt);

// controller
using pros::ADIPneumatics;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-3, -13}, pros::MotorGearset::green); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 11}, pros::MotorGearset::green); // right motor group - ports 6, 7, 9 (reversed)

// individual motors
pros::Motor track_17(-17, pros::MotorGearset::green);
pros::Motor intake_15(15, pros::MotorGearset::green);
pros::Motor brown_lady(-6, pros::MotorGearset::red);

//Pneumatics
pros::adi::Pneumatics Clamp('A', false);
pros::adi::Pneumatics Doinker('C', false);

// Inertial Sensor on port 10
pros::Imu imu(5);

// tracking wheels
// horizontal tracking wheel encoder
pros::Rotation horizontal_sensor(-7);
// vertical tracking wheel encoder
pros::Rotation vertical_sensor(-9);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::OLD_325, 1.625);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::OLD_325, 6.625);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.45, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using green 3.25 inch omnis
                              400, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings lateralController(14, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            145, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(8, // proportional gain (kP)
                                             .05, // integral gain (kI)
                                             90, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

//sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateralController,
                        angularController,
                        sensors,
                        &throttleCurve, 
                        &steerCurve
);

// Functions for the non drive motors and pistons

bool clamp_lock = true;
bool intake_lock = true;
bool track_lock = true;
bool lb_extension = false;
bool lb_start_pos = true;
bool lb_lineup = false;

void clamp() {
    if (clamp_lock) {
        Clamp.extend();
        clamp_lock = false;
    } else {
        Clamp.retract();
        clamp_lock = true;
    }
}

void intake() {
    if (intake_lock) {
        intake_15.move(127);
        intake_lock = false;
    } else {
        intake_15.brake();
        intake_lock = true;
    }
}

void track() {
    if (track_lock) {
        track_17.move(127);
        track_lock = false;
    } else {
        track_17.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        track_17.brake();
        track_lock = true;
    }
}
void bl_uppies() {
    if (lb_start_pos) {
        brown_lady.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        brown_lady.move_absolute(500, 70);
        lb_start_pos = false;
        lb_lineup = true;
    } else if (lb_lineup) {
        brown_lady.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        brown_lady.move_absolute(130, 70);
        lb_lineup = false;
        lb_extension = true;
    } else if (lb_extension) {
        brown_lady.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        brown_lady.move_absolute(0, 70);
        lb_lineup = false;
        lb_extension = true;
    } else {
        brown_lady.brake();
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Vertical Maybe works: %i", vertical_sensor.get_position()); //rotation sensor
            pros::lcd::print(4, "Horizontal (Kinda unecessary): %i", horizontal_sensor.get_position()); //rotation sensor
            pros::lcd::print(5, "14 / 0 / 135"); //Horizontal kd and kp
            pros::lcd::print(6, "LB Position: %i", brown_lady.get_position());
            pros::delay(10); // delay to save resources. DO NOT REMOVE
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //set position to x:0, y:0, heading:0
    brown_lady.set_zero_position(0);
    chassis.setPose(0, 0, 0);
    bl_uppies();
    pros::delay(1200);
    track();
    pros::delay(2000);
	chassis.follow(first_pylon_1_txt, 14, 8000);
    pros::delay(2000);
    clamp();
    chassis.turnToHeading(0, 2000);
    intake();
    pros::delay(600);
	chassis.follow(five_ring_2_txt, 15, 20000);
    pros::delay(2000);
    clamp();
    pros::delay(4000); //Pure Pursuit to next Pylon
    clamp();
    //Pure Pursuit to 5 rings and corner


/** turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
*/
    // turn to face heading 90 with a very long timeout
    //chassis.moveToPose(0, 24, 90, 4000);
}

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            pros::delay(100);
            clamp();
            pros::delay(100);
        }
    }
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            pros::delay(100);
            intake();
            pros::delay(100);
        }
    }
}