#include "main.h"
#include "lemlib/api.hpp"
#include <iostream>
#include <fstream>


#define LEFT_FRONT_PORT       21
#define LEFT_MIDDLE_PORT      8
#define LEFT_BACK_PORT        7
#define RIGHT_FRONT_PORT      18
#define RIGHT_MIDDLE_PORT     11
#define RIGHT_BACK_PORT       13
#define INERTIAL_SENSOR_PORT  9
#define CA_PORT               1
#define INTAKE_PORT           6


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(LEFT_FRONT_PORT, pros::E_MOTOR_GEARSET_06 ,true); // left front motor.  Tracking Wheel 
pros::Motor lM(LEFT_MIDDLE_PORT, pros::E_MOTOR_GEARSET_06,true); // left middle motor. port 11, reversed
pros::Motor lB(LEFT_BACK_PORT, pros::E_MOTOR_GEARSET_06,true); // left back motor. port 1, reversed
pros::Motor rF(RIGHT_FRONT_PORT, pros::E_MOTOR_GEARSET_06,false); // right front motor.  Tracking Wheel 
pros::Motor rM(RIGHT_MIDDLE_PORT, pros::E_MOTOR_GEARSET_06,false ); // right middle motor. port 11
pros::Motor rB(RIGHT_BACK_PORT, pros::E_MOTOR_GEARSET_06,false ); // right back motor. port 13

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

pros::Motor_Group leftTrackingWheelMotors({lF}); // left motor group
pros::Motor_Group righTrackingWheelMotors({rF}); // left motor group


// Inertial Sensor on port 2
pros::Imu imu(INERTIAL_SENSOR_PORT);

// tracking wheels

lemlib::TrackingWheel left_tracking_wheel(&leftTrackingWheelMotors, 3.25, -5.6875,360); // 2.75" wheel diameter, -4.6" offset from tracking center
// right tracking wheel
lemlib::TrackingWheel right_tracking_wheel(&righTrackingWheelMotors, 3.25, 5.6875,360); // 2.75" wheel diameter, 1.7" offset from tracking center



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(&left_tracking_wheel, // vertical tracking wheel 1, set to null
                            &right_tracking_wheel, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

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
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
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
ASSET(example_txt); // '.' replaced with "_" to make c++ happy
//ASSET(path_txt); // '.' replaced with "_" to make c++ happy
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

   //readFile2();

    // example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
      chassis.follow("/usd/path.txt", 2000, 15);
    //chassis.moveToPose(20, 15, 90, 4000);
    // example movement: Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
   // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has travelled 10 inches
   // chassis.waitUntil(10);
   // chassis.cancelMotion();
    // example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
   // chassis.turnTo(45, -45, 1000, true, 60);
    // example movement: Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
   // chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has travelled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    //chassis.waitUntil(10);
   // pros::lcd::print(4, "Travelled 10 inches during pure pursuit!");
    // wait until the movement is done
    //chassis.waitUntilDone();
   pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}


void readFile () {

    FILE* usd_file_read = fopen("/usd/aleena.txt", "r");
    char buf[50]; // This just needs to be larger than the contents of the file
    fread(buf, 1, 50, usd_file_read); // passing 1 because a `char` is 1 byte, and 50 b/c it's the length of buf
  
     pros::lcd::print(5, "%s", buf);
  
    // Should print "Example text" to the terminal
    fclose(usd_file_read); // 

}

void readFile2 () {
    
    
    std::string line;
    std::ifstream file("/usd/aleena.txt");
    if (file.is_open()) {
        while (getline(file, line)) {
           
        }
        file.close();
    }

    const char* str = line.c_str(); 
    pros::lcd::print(5, "%s", str);

}





void writeFile(){

     FILE* usd_file_write = fopen("/usd/aleena.txt", "w");
    fputs("Example text", usd_file_write);
    fclose(usd_file_write);
}
