#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// ==================================== devices ====================================
// create motors and motor groups
pros::MotorGroup left_motors({-5, -6, -7}, pros::MotorGearset::blue); // left motors on ports 10, 18, 19
pros::MotorGroup right_motors({2, 3, 4}, pros::MotorGearset::blue); // right motors on ports 1, 11, 12
pros::MotorGroup lift_motors({13,-14}, pros::MotorGearset::blue); //Lift motors on ports 13 and 14
pros::Motor claw(12, pros::MotorGearset::green); //banshee motor (positive = intake, negative = outtake)
pros::Motor wrist(11, pros::MotorGearset::green); //wrist motor (-300 deg. = upright, 0 deg = down)
pros::Motor intake(8, pros::MotorGearset::blue); //intake motor (negative = intake, positive = reverse)

//create pneumatics
pros::adi::DigitalOut tipperMech ('A');

//create 3-wire and smart port sensors
pros::adi::DigitalIn liftLimit('F');
pros::adi::DigitalIn wristLimit('G');

// create controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// create an imu on port 15
pros::Imu imu(15);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

//drivetrain sensors
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
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
            claw.move(0);
            pros::lcd::print(3, "Running Red Auton");
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            
            lift_motors.move_relative(500, 127); //lift up by a minor amount to prevent hitting the ramp
            wrist.move_absolute(-1080, 127);
            lift_motors.move_relative(0, 127); 
            chassis.moveToPose(35, -10, -90, 1000, {.forwards = false,.maxSpeed = 127}, false); // example: move to (1.0, 1.0, 0.0) with a 5-second timeout
            pros::delay(1000);
            claw.move(-127);
            pros::delay(500);

            chassis.moveToPoint(-3, -10, 1000, {.forwards = true,.maxSpeed = 127}, false);
            chassis.moveToPose(-3, -10, 0, 1000, {.forwards = true,.maxSpeed = 127}, false);



            break;
        case 1:
        chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Blue Auton");
            break;
        case 2:
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Blue Auton");
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
bool clawPosition = false;
bool tipperPosition = false;

void opcontrol() {
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
            lift_motors.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){ //lower
            lift_motors.move(-127);
        } else {
            lift_motors.move(0);
        }

        // intake and claw
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){ //intake
            // spin both claw and intake at once to ensure smooth flow
            // essentially a motor group, but separate for autos
            claw.move(127); //move forward
            intake.move(-127);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ //outtake
            claw.move(-127); //move reverse
            intake.move(127);
        } else {
            claw.move(0); //no dont move
            intake.move(0);
        }

        //make a short macro for toggling scoring position
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){ // whever pressed down
            if (clawPosition == false){ //if wrist is down (false) -> rotate wrist up
                
                lift_motors.move_relative(500, 127); //lift up by a minor amount to prevent hitting the ramp
                wrist.move_absolute(-1070, 127); //flip wrist upright for scoring

                clawPosition = true; // set to other position (scoring position)
                
            } else if (clawPosition == true){ //if wrist is up (true) -> rotate wrist down
                if(lift_motors.get_position() < 500){

                    lift_motors.move_relative(500, 127); //make sure the ramp isn't hit
                    pros::delay(50); // wait for the lift to move before flipping the wrist
                    wrist.move_absolute(0, 127); //flip wrist down for intaking

                } else if (lift_motors.get_position() >= 500){

                    wrist.move_absolute(0, 127); //flip wrist down for intaking

                }
                clawPosition = false; //set to other position (intaking position)
            }
            pros::delay(100); // delay to debounce
        }

        //make a button toggle the tipper mech piston
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            if (tipperPosition == false) {
                tipperMech.set_value(false); // push piston down
                tipperPosition = true; //swap to down position
            } else if (tipperPosition == true) {
                tipperMech.set_value(true); // push piston up
                tipperPosition = false; //swap to up position
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