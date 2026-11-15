#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// ==================================== devices ====================================
// create motors and motor groups
pros::MotorGroup left_motors({-5, -6, -7}, pros::MotorGearset::blue); // left motors on ports 10, 18, 19
pros::MotorGroup right_motors({2, 3, 4}, pros::MotorGearset::blue); // right motors on ports 1, 11, 12
pros::MotorGroup lift_motors({-12, -13, 14, 15}, pros::MotorGearset::green);
    //motors on ports 12, 13, 14, 15 => 12 and 13 on one side, 14 and 15 on other side
    //changed numbers around cuz positive = lift upwards and negative = lift downwards

//create pneumatics
pros::adi::DigitalOut clawPiston ('A');

//create 3-wire and smart port sensors
/* nuke these cuz they aint on the robot but kept them in cuz i MIGHT add them later
pros::adi::DigitalIn liftLimit('F');
pros::adi::DigitalIn wristLimit('G');
*/

//create rotation sensor for the lift
pros::Rotation liftEncoder(16); //port 16

//create distance sensor on claw for autoclaw shenanigans
pros::Distance clawSensor(11); //port 11

// create controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// create an imu on port 15
pros::Imu imu(15);

// create the odom pod/tracking wheel
pros::Rotation vertical_encoder(8);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10.489, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

//drivetrain sensors
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD) // was 3
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              22, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// add driver control curves to make it smoother to drive
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  3.000 // expo curve gain
);

// finally create the chassis with all settings
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

//yeah this function has to be here it stupid
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "tee hee that tickles");
	} else {
		pros::lcd::clear_line(2);
	}
}

// ==================================== initialize ====================================
// auton selector is made here
int autonSelection = 0;

void initialize(){                                                                                                                                
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(on_center_button);
    chassis.calibrate(); // Calibrates properly

    // 1. FIXED CONDITION: Only block out if actually in an official match autonomous or disabled
    while (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
        
        // Handle Selection Cycling
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            autonSelection = autonSelection + 1; 
            if (autonSelection >= 3) autonSelection = 0; 
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonSelection = autonSelection - 1; 
            if (autonSelection < 0) autonSelection = 2;  
        }

        // Display text safely on rows 0 and 1
        pros::lcd::print(0, "Selected Auton ID: %d   ", autonSelection);

        switch (autonSelection) {
            case 0: pros::lcd::print(1, "Selected Right Auton     "); break;
            case 1: pros::lcd::print(1, "Selected Left Auton    "); break;
            case 2: pros::lcd::print(1, "Selected Skills Auton  "); break;
            default: pros::lcd::print(1, "Selected Default Auton "); break;
        }

        // TESTING TRIGGER: Press A to immediately run Auton
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::lcd::print(2, "RUNNING TEST AUTON NOW!  ");
            pros::delay(1000); 
            autonomous();      
            break;             
        }

        // PRACTICE DRIVING TRIGGER: Press B to proceed straight to opcontrol
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            pros::lcd::print(2, "BYPASSING TO DRIVER MODE ");
            pros::delay(200); 
            break;             
        }

        pros::delay(20); // Keep at 20ms to match controller polling rates
    }

    //print the odometry
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(4, "X: %f", chassis.getPose().x); 
            pros::lcd::print(5, "Y: %f", chassis.getPose().y); 
            pros::lcd::print(6, "Theta: %f", chassis.getPose().theta); 
            pros::delay(50); // Pushing tracking info data too quickly can stall screen updates
        }
    });
}

// ==================================== random functions no one cares about ====================================
void disabled() {}

void competition_initialize() {}

// ==================================== autonomous ====================================
void autonomous() {
    switch (autonSelection) {
        
        case 0: //right auto
            chassis.setPose(0, 0, 0); // set initial pose to x = 0, y = 0, theta = 0
            pros::lcd::print(3, "Running Right Auton");

            lift_motors.move_absolute(300, 127); // Move lift up 100 degrees at full speed
            clawPiston.set_value(true); // Close claw
            pros::delay(500); // Wait for 500ms to ensure lift and claw actions
            chassis.moveToPoint(0, 36, 2000); // Move to point (0, 36) within 2000ms    
            pros::delay(500); // Wait for 500ms to ensure movement is complete
            lift_motors.move_absolute(0, 75); // Move lift down to 0 degrees at full speed
            pros::delay(500); // Wait for 500ms to ensure lift action is complete
            clawPiston.set_value(false); // Open claw

            break;

        case 1: //left auto
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Left Auton");

            break;

        case 2: //skills auto
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Skills Auton");

            break;

        default: //default
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Default Auton");

            break;
    }

}

// ==================================== driver control ====================================
bool clawPosition = false;

void opcontrol() {
    lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
        // ^set the motors to hold so that it keeps height

    // ------------------- *record scratch* ok quick controls recap -------------------
    // joysticks -> drive (duh)
    // L1 -> lift up
    // L2 -> lift down
    // down -> claw open/close (toggle)
    // a -> run autonomous code
    // b -> skip to driver control
    // left/right -> cycle through auto selector
    // ------------------- ok back to the code *dramatic sound effect* -------------------
    
    while (true) {
        // ==================================== joystick control ====================================
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // ==================================== button controls/macro ====================================
        // lift
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { //lift 
            lift_motors.move(127); //raise the lift at full speed
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ //lower lift
            lift_motors.move(-100); //make this slower to ensure that the rope doesnt explode and gunk the cascade
        } else {
            lift_motors.move(0); //stop! stop! stop!
        }

        // intake and claw
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){ //intake
            clawPosition = !clawPosition; // swap between states
            clawPiston.set_value(clawPosition); // use that state to toggle the piston
        }


        // delay to save resources
        pros::delay(25);
    }
}