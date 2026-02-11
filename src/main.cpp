#include "main.h"
#include "lemlib/api.hpp" 
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-12, -17, 16},pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({18, 19, -20}, pros::MotorGearset::blue); 

pros::Motor LowerIntake(-10);

pros::Motor UpperIntake(2);

pros::Imu imu(-13);

pros::Rotation VertTrack(5);

pros::Rotation HorizTrack(-8);

lemlib::TrackingWheel horizontal_tracking_wheel(&HorizTrack, lemlib::Omniwheel::NEW_2, 7);//Adjust after build

lemlib::TrackingWheel vertical_tracking_wheel(&VertTrack, lemlib::Omniwheel::NEW_2, 2);//Adjust After Build

pros::adi::DigitalOut Scraper('A');

pros::adi::DigitalOut Midscore('E');

pros::adi::DigitalOut DeScore('F');



lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, 
                              450, // drivetrain rpm is 
                              2// horizontal drifts 2. If we had traction wheels, it would have been 8
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
lemlib::OdomSensors sensors(nullptr, 
                            nullptr, 
                            nullptr, 
                            nullptr, 
                            &imu 
);

lemlib::ExpoDriveCurve throttleCurve(3, 
                                     10, 
                                     1.019 
);


lemlib::ExpoDriveCurve steerCurve(3,
                                  10, 
                                  1.019 
);


lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);






void initialize() {
    pros::lcd::initialize(); 
    chassis.calibrate(); 
    imu.reset();
    pros::delay(1000);
    
    pros::Task screenTask([&]() {
        while (true) {
            
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            
            pros::delay(50);
        }
    });
}



void disabled() {}


void competition_initialize() {}

void autonomous() {
    

    ///Left Goal Rush///
    
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(0, 37, 2000, {.earlyExitRange = 2});
    chassis.moveToPoint(-20, 25, 2000, {.forwards = false, .earlyExitRange = 2});
    chassis.moveToPoint(-23, 45, 2000,{.forwards = false});
    pros::delay(1500);
    UpperIntake.move(127);
    pros::delay(2500);
    UpperIntake.move(0);
    chassis.moveToPoint(-5, 40, 3000, {.earlyExitRange = 2});
    DeScore.set_value(false);
    chassis.moveToPoint(-11, 45, 3000, {.forwards = false, .earlyExitRange = 2});
    chassis.moveToPoint(-12, 70, 3000, {.forwards = false});
    */

    ///Left Split///
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(-7, 20, 2000, /*{ .earlyExitRange = 2}*/);
    chassis.moveToPoint(-7, 30, 2000, /*{.earlyExitRange = 2}*/);
    chassis.turnToHeading(225, 2000);
    chassis.moveToPoint(5, 34, 2000, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(40);
    pros::delay(700);
    UpperIntake.move(0);
    pros::delay(200);
    chassis.moveToPoint(-7, 12, 2000, {.earlyExitRange = 4});
    chassis.moveToPoint(-18, 10, 2000, {.earlyExitRange = 2});
    Scraper.set_value(true);
    chassis.moveToPoint(-24, -12, 2500, {.earlyExitRange = 3});
    chassis.moveToPoint(-8, 25, 2500,{.forwards = false});
    pros::delay(1000);
    Scraper.set_value(false);
    UpperIntake.move(127);
    pros::delay(3000);
    UpperIntake.move(0);
}



void opcontrol() {
    
     

       while (true) {
        
       

        


        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        chassis.arcade(leftY, rightX);
        
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			LowerIntake.move(127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            LowerIntake.move(0);
            UpperIntake.move(0);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			LowerIntake.move(-127);
            UpperIntake.move(-127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			LowerIntake.move(127);
            UpperIntake.move(127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			LowerIntake.move(0);
            UpperIntake.move(-0);            
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            Scraper.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            Scraper.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            Midscore.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            Midscore.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            LowerIntake.move(-127);
            UpperIntake.move(-127);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            DeScore.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            DeScore.set_value(true);
        }

        
        
        pros::delay(10);
    }
    
}