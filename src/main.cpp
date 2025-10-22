#include "main.h"
#include "lemlib/api.hpp" 
#include "pros/misc.h"
#include "pros/rtos.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);


pros::MotorGroup leftMotors({-8, -9, -10},pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({4, 2, 1}, pros::MotorGearset::blue); 

pros::Motor LowerIntake(15);

pros::Motor UpperIntake(-11);

pros::Motor Loader(13);

pros::Imu imu(21);

pros::adi::Encoder horizontalEnc('C','D');

pros::adi::DigitalOut Scraper('H');

pros::adi::DigitalOut Lift('G');

pros::Rotation verticalLeft(14);

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -6);

lemlib::TrackingWheel verticalLft(&verticalLeft, lemlib::Omniwheel::NEW_325, -1.5);


 

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
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

    
    pros::Task screenTask([&]() {
        while (true) {
            
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            
            pros::delay(50);
        }
    });
}


void disabled() {}


void competition_initialize() {}




void autonomous() {
   Loader.brake(); 
   LowerIntake.move(127);
   UpperIntake.move(127);
   Loader.move(-10);
   chassis.moveToPose(0, 20, 0, 4000,{ .earlyExitRange = 7});
   chassis.moveToPose(8, 43, 30, 2000);
   pros::delay(500);
   Scraper.set_value(true); 
   chassis.turnToPoint(10, 20, 3000);
   pros::delay(10);
   chassis.moveToPose(10,  27,  180,  4000);
   chassis.moveToPose(29,  -7,  180,  4000,{.minSpeed = 50});
   pros::delay(3000);
   Lift.set_value(true);
   LowerIntake.move(127);
   UpperIntake.move(0);
   chassis.moveToPose(27, 30, 180, 3000, {.forwards = false});
   UpperIntake.move(0);
   pros::delay(1000);
   Scraper.set_value(false);
   Loader.move(127);

}


void opcontrol() {
   Loader.brake();
    while (true) {
        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        chassis.arcade(leftY, rightX);
        
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			LowerIntake.move(127);
            UpperIntake.move(127);
            Loader.move(-10);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            LowerIntake.move(0);
            UpperIntake.move(0);
            Loader.move(-10);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			LowerIntake.move(-127);
            UpperIntake.move(-127);
            Loader.move(-10);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			LowerIntake.move(127);
            UpperIntake.move(127);
            Loader.move(127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			LowerIntake.move(0);
            UpperIntake.move(0);
            Loader.move(-10);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            Scraper.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            Scraper.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            Lift.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            Lift.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            LowerIntake.move(-127);
            UpperIntake.move(-127);
            Loader.move(-127);
        }


        pros::delay(10);
    }
}