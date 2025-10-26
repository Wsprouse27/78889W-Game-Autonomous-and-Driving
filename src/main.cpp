#include "main.h"
#include "lemlib/api.hpp" 
#include "pros/distance.hpp"
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

pros::Distance BallReader(5);

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

int IntakedBalls = 0;
bool ballPreviouslyDetected = false;
void Test(void *param);

void initialize() {
    pros::lcd::initialize(); 
    chassis.calibrate(); 
    
    
    pros::Task screenTask([&]() {
        while (true) {
            
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Distance: %d mm\n", BallReader.get_distance()); // Distance Sensor
            pros::lcd::print(4, "IntakedBalls: %d", IntakedBalls); // Intaked Balls
            pros::lcd::print(4, "IntakedBalls: %d", IntakedBalls); // Intaked Balls
            lemlib::telemetrySink()->info("Proximity value: %ld \n", BallReader.get_distance());
            
            pros::delay(50);
        }
    });
}


void disabled() {}


void competition_initialize() {}




void autonomous() {
    ///SKILLS///
    /*
    chassis.moveToPoint(-30, 0, 3000, {.forwards = false, .minSpeed = 100});
*/
    ///RIGHT AUTON///
   /*
   LowerIntake.move(127);
   Lift.set_value(true);
   Loader.move(-10);
   chassis.moveToPose(0, 20, 0, 4000,{ .earlyExitRange = 7});
   chassis.moveToPose(8, 43, 30, 2000);
   pros::delay(500);
   chassis.turnToHeading(-45, 3000);
   chassis.moveToPose(-6, 42, -45, 4000);
   pros::delay(1000);
   UpperIntake.move(-127);
   LowerIntake.move(-127);
   
   */

   ///JERRY.IO AUTO///
    Lift.set_value(true);
    chassis.moveToPoint(-34.8, -16.8, 1500);
    chassis.turnToPoint(-22, -22.7, 1500);
    chassis.moveToPoint(-22, -22.7, 1500);
    UpperIntake.move(127);
    LowerIntake.move(127);
    Loader.move(-10);
    chassis.turnToPoint(-13.7, -13.6, 1500);
    chassis.moveToPoint(-13.7, -13.6, 1500);
    LowerIntake.move(-127);
    UpperIntake.move(-127);
    chassis.turnToPoint(-47.5, -47.6, 1500);
    chassis.moveToPoint(-47.5, -47.6, 1500);
    UpperIntake.move(127);
    LowerIntake.move(127);
    Scraper.set_value(true);
    chassis.turnToPoint(-60.1, -47.6, 1500);
    chassis.moveToPoint(-60.1, -47.6, 1500);
    chassis.moveToPoint(-33.2, -47.6, 1500, {.forwards = false});
    UpperIntake.move(127);
    LowerIntake.move(127);
    Loader.move(127);


}

void Test(void *param) {
    
        while(true){
        if (BallReader.get_distance() < 150){
            IntakedBalls = IntakedBalls + 1;
            pros::delay(500);
        }
        if (UpperIntake.get_voltage() > 1 && IntakedBalls >= 4) {
            UpperIntake.move(0);
        }
        pros::delay(10);
        }
}

void opcontrol() {
    
      pros::Task BallTask(Test,NULL);

       while (true) {
        
       

        


        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        chassis.arcade(leftY, rightX);
        
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			LowerIntake.move(127);
            Loader.move(-10);
            if (IntakedBalls < 4) {
                UpperIntake.move(127);
            } else {
                UpperIntake.move(0);
            }
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            LowerIntake.move(0);
            UpperIntake.move(0);
            Loader.move(-10);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			LowerIntake.move(-127);
            UpperIntake.move(-127);
            Loader.move(-10);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            IntakedBalls = 0;
			LowerIntake.move(127);
            UpperIntake.move(127);
            Loader.move(127);
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            IntakedBalls = 0;
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