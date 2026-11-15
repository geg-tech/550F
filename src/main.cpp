#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// ==================================== devices ====================================
// create motors and motor groups
pros::MotorGroup left_motors({-5, -6, -7}, pros::MotorGearset::blue); // left motors on ports 10, 18, 19
pros::MotorGroup right_motors({2, 3, 4}, pros::MotorGearset::blue); // right motors on ports 1, 11, 12
pros::MotorGroup lift_motors({12, 13, -14,-15}, pros::MotorGearset::blue); //Lift motors on ports 13 and 14
    // motors 11 and 12 are on the same side but reversed cuz gears, same for 13 and 13

    

/* redundant code, since removed from robot
pros::Motor claw(12, pros::MotorGearset::green); //banshee motor (positive = intake, negative = outtake)
pros::Motor wrist(11, pros::MotorGearset::green); //wrist motor (-300 deg. = upright, 0 deg = down)
pros::Motor intake(8, pros::MotorGearset::blue); //intake motor (negative = intake, positive = reverse)
*/

//create pneumatics
pros::adi::DigitalOut clawPiston ('A');

//create 3-wire and smart port sensors
/* redundant code, since removed from robot
pros::adi::DigitalIn liftLimit('F');
pros::adi::DigitalIn wristLimit('G');
*/

// create controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// create an imu on port 15
pros::Imu imu(15);

//create a tracking wheel on port 8
pros::Rotation vertical_tracking(8);
lemlib::TrackingWheel vertical_odom(&vertical_tracking, lemlib::Omniwheel::NEW_2, 0); //last number is offset left/right from the center
    //since the wheel is perfectly centered with the robot, no offset is needed

//create rotation sensors for the lift
pros::Rotation lift_rotation(16);

//create distance sensor for the claw
pros::Distance claw_distance(11);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10.489, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

//drivetrain sensors
lemlib::OdomSensors sensors(&vertical_odom, // vertical tracking wheel 1, set to null
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
            if (autonSelection >= 4) autonSelection = 0; 
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonSelection = autonSelection - 1; 
            if (autonSelection < 0) autonSelection = 3;  
        }

        // Display text safely on rows 0 and 1
        pros::lcd::print(0, "Selected Auton ID: %d   ", autonSelection);

        switch (autonSelection) {
            case 0: pros::lcd::print(1, "Selected Red Auton     "); break;
            case 1: pros::lcd::print(1, "Selected Blue Auton    "); break;
            case 2: pros::lcd::print(1, "Selected Yellow Auton  "); break;
            case 3: pros::lcd::print(1, "Selected Skills Auton  "); break;
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
        
        case 0:
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0);
            pros::lcd::print(3, "Running Red Auton");
            break;
        case 1:
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Blue Auton");
            break;
        case 2:
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Yellow Auton");
            break;
        case 3:
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Skills Auton");
            break;
        default:
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Default Auton");
            break;
    }

}

// ==================================== driver control ====================================
bool clawState = false;

void opcontrol() {
    lift_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (true) {
        // ==================================== joystick control ====================================
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // ==================================== button controls/macros ====================================
        // lift
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { //lift upwards
            lift_motors.move(-127); //made this negative cuz new winch geometry
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ //lower lift
            lift_motors.move(100); //slightly slower than up to make the rope not explode itself
        } else {
            lift_motors.move(0);
        }

        //make a button toggle the claw piston
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            if (clawState == false) {
                clawPiston.set_value(false); // push piston down
                clawState = true; //swap to down position
            } else if (clawState == true) {
                clawPiston.set_value(true); // push piston up
                clawState = false; //swap to up position
            }
        }


        //make a small macro to test PID
        //angular PID
        /*
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){ //niche ahh button
            //reset position
            chassis.setPose(0,0,0);
            //turn
            chassis.turnToHeading(90,4000);
            pros::delay(100);
            chassis.turnToHeading(180,4000);
            pros::delay(100);
            chassis.turnToHeading(0,4000);
            pros::delay(100);
        }
        */



        // delay to save resources
        pros::delay(25);
    }
}