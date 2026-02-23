#include "main.h"
#include "lemlib/api.hpp" 
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

///Device Configuration///
//Controller//
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Drivetrain Setup//
pros::MotorGroup leftMotors({-12, -17, 16},pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({18, 19, -20}, pros::MotorGearset::blue);
pros::Imu imu(-13);

//Intake Setup//
pros::Motor LowerIntake(-10);
pros::Motor UpperIntake(21);

//Pneumatic Devices//
pros::adi::DigitalOut Scraper('A');
pros::adi::DigitalOut Midscore('E');
pros::adi::DigitalOut DeScore('F');

//Light Controllers//
pros::adi::Motor BlueMotor('B');
pros::adi::Motor RedMotor('D');
pros::adi::DigitalIn LEDButton('C');

///Drivetrain Tuning///
//Drivetrain and Sensor Setup//
lemlib::Drivetrain drivetrain(&leftMotors, // Left Motor Group
                              &rightMotors, // Right Motor group
                              12.5, // 12.5 inch Track Width
                              lemlib::Omniwheel::NEW_325, // New 3.25 inch Omni-Wheel
                              450, // Drivetrain RPM
                              2// horizontal drift is 2. If we had traction wheels, it would have been 8
);

//Lateral Motion COntroller
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

//Angular Motion Controller
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

//Sensors Setup for Odometry//
lemlib::OdomSensors sensors(nullptr, //Internal Motor Controllers
                            nullptr, //Internal Motor Controllers
                            nullptr, //Internal Motor Controllers
                            nullptr, //Internal Motor Controllers
                            &imu //Inertial Sensor
);

//Driver Control Tuning (Throttle)//
lemlib::ExpoDriveCurve throttleCurve(3, 
                                     10, 
                                     1.019 
);

//Driver Control Tuning (Steering)//
lemlib::ExpoDriveCurve steerCurve(3,
                                  10, 
                                  1.019 
);

///Final Initilazation for all Drivetrail Related Controlls///
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

//Color Function//
void ColorSet(){
    if (LEDButton.get_value()){
        BlueMotor.set_value(-127);
        RedMotor.set_value(0);
    } else {
        RedMotor.set_value(-127);
        BlueMotor.set_value(0);
    }
}

//Initialization Runs at start of Code
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

//Autonomous Functions
void autonomous() {
    
    ColorSet();
    ///Left Goal Rush///
    
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(0, 37, 2000, {.earlyExitRange = 2});
    chassis.moveToPoint(-20, 25, 2000, {.forwards = false, .earlyExitRange = 2});
    chassis.moveToPoint(-20, 45, 3000,{.forwards = false});
    pros::delay(1500);
    UpperIntake.move(127);
    pros::delay(5000);
    UpperIntake.move(0);
    */
    

    ///Left Split///
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(-12, 30, 2000);
    chassis.turnToPoint(3, 37, 1500, {.forwards = false});
    chassis.moveToPoint(3, 37, 1500, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(40);
    pros::delay(900);
    UpperIntake.move(0);
    pros::delay(200);
    chassis.turnToPoint(-40, 3, 1500, {.minSpeed = 50});
    chassis.moveToPoint(-40, 3, 4000);
    Scraper.set_value(true);
    chassis.turnToPoint(-40, -14, 1500);
    chassis.moveToPoint(-40, -14, 2000,{.minSpeed = 40});
    pros::delay(2500);
    chassis.moveToPoint(-35, 20, 1500, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(127);
    Scraper.set_value(false);
    */

    ///Right 7 High///
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(15, 30, 2000);
    chassis.turnToPoint(41, 0, 1500);
    chassis.moveToPoint(41, 0, 2000,{.maxSpeed = 80});
    chassis.turnToHeading(176, 1500);
    Scraper.set_value(true);
    chassis.moveToPoint(43, -15, 4000, {.maxSpeed = 80});
    pros::delay(2000);
    chassis.moveToPoint(41, 25, 1500, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(127);
    pros::delay(2000);
    UpperIntake.move(0);
    chassis.turnToHeading(270, 1500, {.minSpeed = 127});
    chassis.turnToHeading(270, 1000, {.minSpeed = 127});
    */
    
    BlueMotor.set_value(0);
    RedMotor.set_value(0);

}


//Driver Control Function runs during Driver Control
void opcontrol() {
     
     ColorSet();
        
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
			
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            Scraper.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            Scraper.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            Midscore.set_value(true);
            UpperIntake.move(60);
            LowerIntake.move(127);
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