#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// ==================================== devices ====================================
// create motors and motor groups
pros::MotorGroup left_motors({-2, -3, -4}, pros::MotorGearset::blue); // left motors on ports 10, 18, 19
pros::MotorGroup right_motors({5, 6, 7}, pros::MotorGearset::blue); // right motors on ports 1, 11, 12
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
pros::Rotation liftEncoder(-16); //port 16

//create distance sensor on claw for autoclaw shenanigans
pros::Distance clawSensor(11); //port 11

// create controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// create an imu on port 17
pros::Imu imu(17);

// create the odom pod/tracking wheels
// X and Y follow the same layout of a standard cartesian plane (+x to the right, +y forwards from the driver box)
pros::Rotation vertical_encoder(8);
pros::Rotation horizontal_encoder(-18); //reverse direction so y position is right
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.8);


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
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
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

// create jerry.io paths for auto
// pls ingore i was just messing around with this
ASSET(pathjerryio_txt);
ASSET(gegpath_txt);

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
            pros::lcd::print(7, "Lift Encoder: %f", liftEncoder.get_position());
            pros::delay(50); // Pushing tracking info data too quickly can stall screen updates
        }
    });
}

// ==================================== random functions no one cares about ====================================
void disabled() {}

void competition_initialize() {}

int allianceHeights[5] = {1000, 1500, 2000, 2500, 3000}; // array to store alliance heights for each auton
int neutralHeights[5] = {1000, 1500, 2000, 2500, 3000}; // array to store neutral heights for each auton
int centerHeights[3] = {1500, 2000, 2500}; // array to store center heights for each auton

int heightVar = 0;
//this function uses lift encoder to move the lift to designated height, more consistent then motor encoders
// void liftToHeight(int targetHeight, int speed) {
//     int currentHeight = liftEncoder.get_position(); // get the current height of the lift
//     if (currentHeight < targetHeight) { // if the lift is below the target height
//         while (liftEncoder.get_position() < targetHeight) { // while the lift is below the target height
//             lift_motors.move(speed); // move the lift up at designated speed
//             pros::delay(10); // delay to prevent CPU overload
//         }
//     } else if (currentHeight > targetHeight) { // if the lift is above the target height
//         while (liftEncoder.get_position() > targetHeight) { // while the lift is above the target height
//             lift_motors.move(-speed); // move the lift down at designated speed
//             pros::delay(10); // delay to prevent CPU overload
//         }
//     }
//     lift_motors.move(0); // stop the lift motors when the target height is reached
// }

// ==================================== autonomous ====================================
void autonomous() {
    switch (autonSelection) {
        
        case 0: //right auto
            chassis.setPose(0, 0, 0); // set initial pose to x = 0, y = 0, theta = 0
            pros::lcd::print(3, "Running Right Auton");


            /* old code for reference, remove if needed
            liftToHeight(300, 127); // Move lift up 100 degrees at full speed
            clawPiston.set_value(false); // close claw just to make sure
            pros::delay(500); // Wait for 500ms to ensure lift and claw actions
            chassis.moveToPoint(0, 36, 2000); // Move to point (0, 36) within 2000ms    
            pros::delay(500); // Wait for 500ms to ensure movement is complete
            liftToHeight(0, 75); // Move lift down to 0 degrees at full speed
            pros::delay(500); // Wait for 500ms to ensure lift action is complete
            clawPiston.set_value(false); // close claw
            */

            chassis.moveToPoint(0,15,2000); //move forwards a bit to get away from the wall

            while (heightVar <= 1) {
                lift_motors.move_absolute(allianceHeights[heightVar], 127); //move lift out of the way of the goal
                chassis.moveToPoint(-17,20, 1000, {.maxSpeed = 75}); //move towards the goal
                pros::delay(1750); //delay to simulate scoring and to not implode the robot
                clawPiston.set_value(true); //open the claw to score
                lift_motors.move_absolute(0, 127); //lower the lift to matchload height
                pros::delay(250);
                chassis.moveToPoint(-10,19,500); //get away from the goal really quickly
                if (heightVar == 1){
                    break; //break out of the loop if we are done with the last height
                }
                chassis.moveToPose(13.5,21,-180,2000, {.forwards = false}); //move to a good position to matchload
                chassis.moveToPoint(10., 7.5,  1000); //move to the matchload
                pros::delay(1000); //delay to simulate picking up the matchload
                clawPiston.set_value(false); //close the claw to pick up the matchload
                pros::delay(1000); //delay to simulate picking up the matchload
                chassis.moveToPoint(13.5,21,2000, {.forwards=false}); //move back to a good position to score

                heightVar++; //increment the height variable to move to the next height
            }

            break;

        case 1: //left auto
            //alex's code for testing, please change if needed
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Left Auton");
            chassis.moveToPoint(0,15,2000); // move forwards a bit to get away from the wall

            while (heightVar <= 1) {
                lift_motors.move_absolute(allianceHeights[heightVar], 127); // move lift out of the way of the goal
                chassis.moveToPoint(17,20, 1000, {.maxSpeed = 75}); // move towards the goal
                pros::delay(1750); // delay to simulate scoring and to not implode the robot
                clawPiston.set_value(true); // open the claw to score
                lift_motors.move_absolute(0, 127); // lower the lift to matchload height
                pros::delay(250);
                chassis.moveToPoint(10,19,500); // get away from the goal really quickly
                if (heightVar == 1){
                    break; // break out of the loop if we are done with the last height
                }
                chassis.moveToPose(-13.5,21,180,2000, {.forwards = false}); // mirrored pose
                chassis.moveToPoint(-10, 7.5, 1000); // move to the matchload
                pros::delay(1000); // delay to simulate picking up the matchload
                clawPiston.set_value(false); // close the claw to pick up the matchload
                pros::delay(1000); // delay to simulate picking up the matchload
                chassis.moveToPoint(-13.5,21,2000, {.forwards = false}); // move back to a good position to score

                heightVar++; // increment the height variable to move to the next height
            }

            break;

        case 2: //skills auto
            chassis.setPose(0, 0, 0); // set initial pose to (0, 0, 0)
            pros::lcd::print(3, "Running Skills Auton");

            //testing jerry.io, please ignore
            //chassis.follow(gegpath_txt, 15, 50000); // follow the path with max speed of 127 and no other constraints

            chassis.moveToPoint(0,15,2000); //move forwards a bit to get away from the wall

            while (heightVar <= 4) {
                lift_motors.move_absolute(allianceHeights[heightVar], 127); //move lift out of the way of the goal
                chassis.moveToPoint(-17,20, 1000, {.maxSpeed = 75}); //move towards the goal
                pros::delay(1750); //delay to simulate scoring and to not implode the robot
                clawPiston.set_value(true); //open the claw to score
                lift_motors.move_absolute(0, 127); //lower the lift to matchload height
                pros::delay(250);
                chassis.moveToPoint(-10,22,500); //get away from the goal really quickly
                lift_motors.move_absolute(0, 127);
                chassis.moveToPose(13.5,22,-180,2000, {.forwards = false}); //move to a good position to matchload
                chassis.moveToPoint(12, 6,  1000); //move to the matchload
                pros::delay(1000); //delay to simulate picking up the matchload
                clawPiston.set_value(false); //close the claw to pick up the matchload
                pros::delay(1000); //delay to simulate picking up the matchload
                chassis.moveToPoint(13.5,22,2000, {.forwards=false}); //move back to a good position to score
                if (heightVar == 4){
                    break; //break out of the loop if we are done with the last height
                }
            
                heightVar++; //increment the height variable to move to the next height
            }

            heightVar = 0; //reset the height variable for center goal scoring
            pros::delay(1000); //delay to simulate scoring and to not implode the robot
            while (heightVar <= 2) {
                lift_motors.move_absolute(centerHeights[heightVar],127); //move lift out of the way of the goal
                chassis.moveToPoint(-56, 56, 7000, {.maxSpeed=50}); //move to a good position to score the center goal
                pros::delay(1000); //delay to simulate scoring and to not implode the robot
                clawPiston.set_value(true); //open the claw to score
                lift_motors.move_absolute(0, 127); //lower the lift to matchload height
                pros::delay(250);
                chassis.moveToPoint(-50,50,500, {.forwards=false});
                if (heightVar == 2){
                    break; //break out of the loop if we are done with the last height
                }
                chassis.moveToPose(13.5,22,-180,2000, {.forwards = false}); //move to a good position to matchload
                chassis.moveToPoint(12, 6,  1000); //move to the matchload
                pros::delay(1000); //delay to simulate picking up the matchload
                clawPiston.set_value(false); //close the claw to pick up the matchload
                pros::delay(1000); //delay to simulate picking up the matchload
                chassis.moveToPoint(13.5,22,2000, {.forwards=false}); //move back to a good position to score

                heightVar++; //increment the height variable to move to the next height
            }
            
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