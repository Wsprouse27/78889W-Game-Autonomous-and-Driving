#include "main.h"
#include "lemlib/api.hpp" 
#include "lemlib/chassis/chassis.hpp"
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
pros::Imu imu(-4);
pros::Rotation HorizontalEncoder(-8);
lemlib::TrackingWheel HorizontalTracking(&HorizontalEncoder, lemlib::Omniwheel::NEW_2, 6.5);

//Intake Setup//
pros::Motor LowerIntake(-10);
pros::Motor UpperIntake(-21);

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
                                            6, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

//Angular Motion Controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             30, // derivative gain (kD)
                                             15, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

//Sensors Setup for Odometry//
lemlib::OdomSensors sensors(nullptr, //Internal Motor Controllers
                            nullptr, //Internal Motor Controllers
                            &HorizontalTracking, //Horizontal Tracking Wheel
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
    
    //chassis.setPose(0,0,0);
    //chassis.turnToHeading(90, 2000);


    //ColorSet();
    ///Left 7///
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(-15, 30, 2000);
    chassis.turnToPoint(-38, 0, 1500);
    chassis.moveToPoint(-40, 0, 2000,{.maxSpeed = 80});
    chassis.turnToHeading(180, 1500);
    Scraper.set_value(true);
    chassis.moveToPoint(-40, -16, 4000);
    pros::delay(1500);
    chassis.moveToPoint(-38, 25, 1500, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(127);
    pros::delay(3000);
    UpperIntake.move(0);
    Scraper.set_value(false);
    chassis.moveToPoint(-38, 0, 2000);
    chassis.turnToHeading(220, 1000);
    DeScore.set_value(false);
    chassis.moveToPoint(-23, 25, 2000, {.forwards = false});
    chassis.moveToPoint(-23, 35, 2000, {.forwards = false});
    */

    ///Left Split///
    
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(-10, 20, 1500);
    chassis.turnToHeading(220, 800);
    chassis.moveToPoint(5, 28, 1500, {.forwards = false});
    Midscore.set_value(true);
    pros::delay(200);
    LowerIntake.move_relative(-80, 70);
    UpperIntake.move_relative(-80, 70);
    UpperIntake.move(-127);
    LowerIntake.move(127);
    pros::delay(800);
    UpperIntake.move(0);
    Midscore.set_value(false);
    chassis.moveToPoint(-35, 0, 1500/*, {.maxSpeed = 70}*/);
    chassis.turnToHeading(170, 800);
    Scraper.set_value(true);
    chassis.moveToPoint(-35, -16, 2500, {.minSpeed = 90});
    pros::delay(1200);
    chassis.moveToPoint(-33, 20, 1500, {.forwards = false});
    pros::delay(900);
    UpperIntake.move(127);
    
    
    /*
    pros::delay(1000);
    UpperIntake.move(0);
    DeScore.set_value(false);
    chassis.moveToPoint(-25, 10, 2000);
    chassis.turnToHeading(180, 2000);
    */
    

    ///Right 7 High///
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(15, 30, 2000);
    chassis.turnToPoint(41, 0, 1500);
    chassis.moveToPoint(41, 0, 2000,{.maxSpeed = 80});
    chassis.turnToHeading(185, 1500);
    Scraper.set_value(true);
    chassis.moveToPoint(41, -12, 4000);
    pros::delay(2000);
    chassis.moveToPoint(41, 25, 1500, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(127);
    pros::delay(3000);
    UpperIntake.move(0);
    Scraper.set_value(false);
    chassis.moveToPoint(41, 0, 2000);
    DeScore.set_value(false);
    chassis.turnToHeading(200, 2000);
    chassis.moveToPoint(50, 25, 2000, {.forwards = false});
    chassis.moveToPoint(50, 35, 2000, {.forwards = false});
    */

    ///Left 4 Push///
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(-15, 30, 2000);
    chassis.turnToPoint(-39, 0, 1500);
    chassis.moveToPoint(-39, 0, 2000,{.maxSpeed = 80});
    chassis.turnToHeading(-185, 1500);
    chassis.moveToPoint(-39, 25, 1500, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(127);
    pros::delay(2000);
    UpperIntake.move(0);
    DeScore.set_value(false);
    chassis.moveToPoint(-39, 5, 2000);
    chassis.turnToHeading(-210, 2000);
    chassis.moveToPoint(-43, 25, 2000, {.forwards = false});
    chassis.turnToHeading(-170, 2000);
    chassis.moveToPoint(-43, 40, 2000);
    */

    ///Rigth 4 Push///
    /*
    chassis.setPose(0,0,0);
    LowerIntake.move(127);
    DeScore.set_value(true);
    chassis.moveToPoint(15, 30, 2000);
    chassis.turnToPoint(38, 0, 1500);
    chassis.moveToPoint(38, 0, 2000,{.maxSpeed = 80});
    chassis.turnToHeading(185, 1500);
    chassis.moveToPoint(38, 25, 1500, {.forwards = false});
    pros::delay(1000);
    UpperIntake.move(127);
    pros::delay(2000);
    UpperIntake.move(0);
    DeScore.set_value(false);
    chassis.moveToPoint(38, 5, 2000);
    chassis.turnToHeading(210, 2000);
    chassis.moveToPoint(40, 25, 2000, {.forwards = false});
    chassis.moveToPoint(43, 40, 2000, {.forwards = false});
    */
    






    

    
    
    BlueMotor.set_value(0);
    RedMotor.set_value(0);

}


//Driver Control Function runs during Driver Control
void opcontrol() {
     
     //ColorSet();
        
       while (true) {
        

        


        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        chassis.arcade(leftY, rightX);
        
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			LowerIntake.move(127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            LowerIntake.move(0);
            UpperIntake.move(0);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			LowerIntake.move(127);
            UpperIntake.move(127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            Scraper.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            Scraper.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            LowerIntake.move(-127);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            DeScore.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            DeScore.set_value(true);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            UpperIntake.move(127);
            LowerIntake.move(127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
		Midscore.set_value(true);
        UpperIntake.move(-127);
        LowerIntake.move(127);
		} else {
            Midscore.set_value(false);
            UpperIntake.move(0);
        }
        
        
        pros::delay(10);
    }
    
}