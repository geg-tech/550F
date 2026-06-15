/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ApopkaBeta                                                */
/*    Created:      10/8/2025, 12:54:28 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

  // define controller
controller Controller = controller();

  // define drivetrain motors/motor groups
    // motors
motor MotorL1 = motor(PORT10, ratio18_1,true); //second parameter dictates motor cartridge, ratio6_1 is blue speed cartridge
motor MotorL2 = motor(PORT9, ratio18_1,true);
motor MotorR1 = motor(PORT8, ratio18_1);
motor MotorR2 = motor(PORT7, ratio18_1);

    // motors for Scoop
motor ScoopLeft = motor(PORT5, ratio18_1);
motor ScoopRight = motor(PORT4, ratio18_1, true);

    // motor for intake
motor Intake = motor(PORT20, ratio18_1);

    // motor groups
motor_group DrivetrainLeft = motor_group(MotorL1, MotorL2);
motor_group DrivetrainRight = motor_group(MotorR1, MotorR2);
motor_group Scoopbothsides = motor_group(ScoopLeft, ScoopRight);

  // define drivetrain object
drivetrain Drivetrain = drivetrain(DrivetrainLeft, DrivetrainRight, 12.5, 18, 12, inches, 0.75);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

  // we got smartdrive turnFor at home
  // uses inertial sensor to turn to an abolute heading on the field 
  // (robot initally starts perpendicular to field w/ front facing 0 degrees)


void autonomous(void) {
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    // ........................................................................


         // Drivetrain movement //
       

      // get joystick values
    int leftY = 6 * Controller.Axis1.position(); // multiply percent by 6 to get rpm, Controller.AxisNUMBER.position(); returns numbers -100 to 100
    int rightX = 6 * Controller.Axis3.position();
      // move the left side of the robot
    DrivetrainLeft.spin(forward, leftY + rightX, rpm);

      // move the right side of the robot 
    DrivetrainRight.spin(forward, rightX - leftY, rpm);
    

          // Scoop Stuff //


    // bool for both sides of scoop
    bool pressy = Controller.ButtonL2.pressing();
    bool pressylower = Controller.ButtonL1.pressing();

    // variable fo Left Scoop
    // set up speed for scoop motors
    // if controller button R1 is presse motors spin reversed, if R2 is pressed motors spin out, if else stop
    if (pressy == true){
      ScoopLeft.spin(reverse);
    } else if (pressylower == true) {
      ScoopLeft.spin(forward);
    } else {
      ScoopLeft.stop();
    }

    // variable fo Right Scoop
    bool NotVerypressed= Controller.ButtonR2.pressing();
    bool Verypressed = Controller.ButtonR1.pressing();
    // set up speed for right scoop motor
    
    // if controller bumper is presse motor go spin
    if (Verypressed == true){
      ScoopRight.spin(forward);
    } else if (NotVerypressed == true){
      ScoopRight.spin(reverse);
    } else {
      ScoopRight.stop();
    }

    // variable fo Intake
    bool ExtremelyNotpressed= Controller.ButtonA.pressing();
    // set up speed for Intake
    Intake.setVelocity(200, rpm);

        if (ExtremelyNotpressed == true){
      Intake.spin(reverse);
    } else {
      Intake.stop();
    }
    
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
