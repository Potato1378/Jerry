#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
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

// motor groups
pros::MotorGroup leftMotors({-3, -13}, pros::MotorGearset::green); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 11}, pros::MotorGearset::green); // right motor group - ports 6, 7, 9 (reversed)

// individual motors
pros::Motor track_17(17, pros::MotorGearset::green);
pros::Motor intake_15(-15, pros::MotorGearset::green);
pros::Motor brown_lady(-6, pros::MotorGearset::red);

//Pneumatics
pros::adi::Pneumatics Clamp('A', false);
pros::adi::Pneumatics Doinker('C', false);

// Inertial Sensor on port 10
pros::Imu imu(5);

// tracking wheels
// horizontal tracking wheel encoder
pros::Rotation horizontal_sensor(7);
// vertical tracking wheel encoder
pros::Rotation vertical_sensor(-9);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::OLD_325, -3.25);//-2.85
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::OLD_325, 6.85);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.7, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using green 3.25 inch omnis
                              400, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            45, // derivative gain (kD)
                                            3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                            .004, // integral gain (kI)
                                            16, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
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

bool trackLock = true;
bool intakeLock = true;
bool doink_lock = true;
bool clamp_lock = true;

bool lb_extension = false;
bool lb_start_pos = true;
bool lb_lineup = false;
bool lb_waiting = false;

void blah() {
    if (clamp_lock) {
        Clamp.extend();
        clamp_lock = false;
    } else {
        Clamp.retract();
        clamp_lock = true;
    }
}

void clamp() {
    if (clamp_lock) {
        Clamp.extend();
    } else {
    }
clamp_lock = false;
}

void unClamp() {
    if (clamp_lock == false) {
        Clamp.retract();
    } else {
    }
clamp_lock = true;
}

void intakeUp() {
    if (intakeLock) {
        intake_15.move(110);
    } else {
        intake_15.brake();
    }
    if (trackLock) {
        track_17.move(125);
    } else {
        track_17.brake();
    }
intakeLock = !intakeLock;
trackLock = !trackLock;

}

void intakeDown() {
    if (intakeLock) {
        intake_15.move(-110);
    } else {
        intake_15.brake();
    }
    if (trackLock) {
        track_17.move(-125);
    } else {
        track_17.brake();
    }
intakeLock = !intakeLock;
trackLock = !trackLock;
}

void doinker() {
    if (doink_lock) {
        Doinker.extend();
        doink_lock = false;
    } else {
        Doinker.retract();
        doink_lock = true;
    }
}

void bl_uppies() {
    if (lb_start_pos) {
        brown_lady.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        track_17.brake();
        track_17.set_zero_position(0, 0);
        track_17.move_absolute(-250, 70);
        brown_lady.move_absolute(525, 70);
        lb_start_pos = false;
        lb_lineup = true;
    } else if (lb_lineup) {
        brown_lady.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        brown_lady.move_absolute(750, 70);
        lb_lineup = false;
        lb_extension = true;
    } else if (lb_extension) {
        brown_lady.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        brown_lady.move_absolute(525, 70);
        lb_extension = false;
        lb_waiting = true;
    } else if (lb_waiting) {
        brown_lady.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        brown_lady.move_absolute(0, 70);
        lb_waiting = false;
        lb_start_pos = true;
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
            pros::lcd::print(3, "Vertical: %i", vertical_sensor.get_position()); //rotation sensor
            pros::lcd::print(4, "Horizontal: %i", horizontal_sensor.get_position()); //rotation sensor
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
    track_17.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake_15.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    bl_uppies();
    pros::delay(400);
    intakeUp();
    pros::delay(800);
	chassis.moveToPose(3, 13.5, 90, 2400);
    chassis.moveToPose(-25, 10.5, 90, 2200, {.forwards = false});
    pros::delay(1600);
    clamp();

    chassis.moveToPose(-25, 35, 305, 3500, {.minSpeed = 30, .earlyExitRange = .3});
    chassis.moveToPose(-55, 37.5, 267, 3500, {.minSpeed = 35, .earlyExitRange = .3});
    chassis.turnToHeading(180, 1800);
    chassis.moveToPose(-51, -8, 180, 4000, {.minSpeed = 35});

    chassis.turnToHeading(310, 1100);
    chassis.moveToPose(-62, 18, 310, 2400);
    chassis.moveToPose(-57, 9, 315, 2200, {.forwards = false});
    chassis.turnToHeading(45, 1200);
    chassis.moveToPose(-68, -9, 45, 2200, {.forwards = false});
    pros::delay(1600);
    unClamp();

    chassis.moveToPose(-3, 13.5, 90, 4000);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPose(23.5, 16.25, 270, 2200, {.forwards = false});
    pros::delay(1300);
    clamp();

    chassis.moveToPose(21, 43.5, 55, 3500, {.minSpeed = 30, .earlyExitRange = .3});
    chassis.moveToPose(44, 43.5, 90, 3500, {.minSpeed = 40, .earlyExitRange = .3});
    chassis.turnToHeading(180, 1800);
    chassis.moveToPose(33.5, -7, 180, 4000, {.minSpeed = 30});

    chassis.turnToHeading(40, 900);
    chassis.moveToPose(54, 13, 40, 2400);
    chassis.moveToPose(48, 7, 45, 2400, {.forwards = false});
    chassis.turnToHeading(315, 1200);
    chassis.moveToPose(58.5, -8.5, 315, 2200, {.forwards = false});
    pros::delay(1600);
    unClamp();
/**
    chassis.turnToHeading(270, 1000);
    chassis.moveToPose(25, 10.5, 270, 3000, {.forwards = false});
    clamp();
    chassis.moveToPose(25, 34, 100, 3500, {.minSpeed = 5, .earlyExitRange = .8});
    chassis.moveToPose(50, 36, 90, 3500, {.minSpeed = 5, .earlyExitRange = .8});
    chassis.turnToHeading(180, 1800);
    chassis.moveToPose(48, -2, 180, 4000, {.minSpeed = 5, .earlyExitRange = .8});
*/
/**
    chassis.setPose(48, -2, 180);
    pros::delay(50);
    chassis.swingToHeading(90, DriveSide::RIGHT, 2000, {.direction = AngularDirection::CW_CLOCKWISE});
    chassis.moveToPose(66, 11.5, 90, 2400);
    chassis.swingToHeading(315, DriveSide::LEFT, 2000, {.direction = AngularDirection::CW_CLOCKWISE});
    chassis.moveToPose(66, 4, 315, 2000, {.forwards = false});
    unClamp();
*/
//chassis.moveToPose( 40, 12, 45, 3000);
}

void opcontrol() {
    pros::Controller cont(pros::E_CONTROLLER_MASTER);
    track_17.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake_15.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = cont.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = cont.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            clamp();
            pros::delay(25);
        }
    
        if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            unClamp();
            pros::delay(25);
        }
    
        if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intakeUp();
            pros::delay(25);
        }

        if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            intakeDown();
            pros::delay(25);
        }

        if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            bl_uppies();
            pros::delay(25);
        }

        if (cont.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            doinker();
            pros::delay(25);
        }

        // delay to save resources
        pros::delay(5);
    }
}