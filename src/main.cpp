#include "main.h"
#include "lemlib/api.hpp" 
#include "pros/adi.hpp"
#include "pros/ai_vision.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-8, -9, -10},pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({4, 2, 1}, pros::MotorGearset::blue); 

pros::Motor LowerIntake(-15);

pros::Motor UpperIntake(-11);

pros::Motor Loader(13);

pros::Imu imu(21);

pros::adi::DigitalOut Scraper('H');

pros::adi::DigitalOut Lift('G');

pros::adi::DigitalOut DeScore('F');

pros::Distance BallReader(5);

pros::Distance LeftGoal(12);

pros::Distance RightGoal(19);

 

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
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





void CheckGoal(){
    int HeadingGoal = imu.get_heading();
    pros::delay(10);
    while (RightGoal.get_distance() < 500 || LeftGoal.get_distance() < 500){
        if(RightGoal.get_distance() < 500){

            chassis.turnToHeading(HeadingGoal + 12.5, 500);
        }
        if(LeftGoal.get_distance() < 500){

            chassis.turnToHeading(HeadingGoal - 7, 500);
        }
        pros::delay(50);
    }
}


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

pros::Task BallTask(Test,NULL);

void autonomous() {

   

    

    ///SKILLS///
    /*
        BallTask.suspend();
        chassis.setPose(-46.5,-7.4, 180);
        Scraper.set_value(true);
        Loader.move(127);
        LowerIntake.move(127);
        UpperIntake.move(127);
        DeScore.set_value(true);
        Lift.set_value(true);
        chassis.moveToPoint(-46.5, -47, 2500,{.maxSpeed = 50});
        chassis.turnToHeading(270, 1500);
        chassis.moveToPoint(-62, -47, 1500);
        chassis.moveToPoint(-55, -47, 1500, {.forwards = false});
        chassis.moveToPoint(-62, -47, 1500);
        chassis.moveToPoint(-55, -47, 1500, {.forwards = false});
        chassis.moveToPoint(-62, -47, 1500);
        pros::delay(1500);
        chassis.moveToPoint(-45, -48, 2500,{.forwards = false, .maxSpeed = 50});
        Scraper.set_value(false);
        chassis.turnToPoint(-68, -32, 1500);
        chassis.moveToPoint(-68, -32, 1500);
        chassis.turnToPoint(-72, 20, 1500);
        chassis.moveToPoint(-72, 20, 1500);
        Scraper.set_value(true);
        pros::delay(1500);
        Scraper.set_value(false);
    */

        

   ///Right AUTO///
   /*
    BallTask.suspend();
    chassis.setPose(-61, -17, 90);
    Lift.set_value(true);
    chassis.moveToPoint(-34.8, -16.8, 1500);
    chassis.turnToPoint(-19.6, -23.2, 1500);
    chassis.moveToPoint(-19.6, -23.2, 1500, {.maxSpeed=30});
    UpperIntake.move(127);
    LowerIntake.move(127);
    Loader.move(-10);
    chassis.turnToPoint(-12.5, -13.4, 1500);
    chassis.moveToPoint(-12.5, -13.4, 1500);
    LowerIntake.move(-60);
    UpperIntake.move(-127);
    pros::delay(3000);
    chassis.moveToPoint(-47.5, -47.5, 1500, {.forwards = false});
    UpperIntake.move(127);
    LowerIntake.move(127);
    chassis.turnToPoint(-61.7, -49, 1500);
    Scraper.set_value(true);
    chassis.moveToPoint(-61.7, -49, 2000);
    pros::delay(1500);
    Scraper.set_value(true);
    chassis.moveToPoint(-30, -46.5, 1500, {.forwards = false});
    UpperIntake.move(127);
    LowerIntake.move(127);
    Loader.move(127);
    */

    ///LEFT AUTON///
    /*
    BallTask.suspend();
    Scraper.set_value(true);
    chassis.setPose(-61.8, 16.7, 90);
    Loader.move(-70);
    chassis.moveToPoint(-38.7, 16.7, 2000);
    chassis.turnToPoint(-22, 21.8, 2000);
    chassis.moveToPoint(-22, 21.8, 1500,{.maxSpeed = 35});
    Scraper.set_value(false);
    LowerIntake.move(127);
    UpperIntake.move(127);
    chassis.turnToPoint(-13.9, 10, 2000, {.forwards = false});
    UpperIntake.move(0);
    chassis.moveToPoint(-13.9, 10, 2000, {.forwards = false});
    UpperIntake.move(127);
    Loader.move(50);
    pros::delay(2500);
    Loader.move(-20);
    chassis.turnToPoint(-47.4, 47, 2000);
    Scraper.set_value(true);
    chassis.moveToPoint(-47.4, 47, 2000);
    chassis.turnToPoint(-63, 47, 2000);
    chassis.moveToPoint(-63, 47, 2000);
    Lift.set_value(true);
    pros::delay(1500);
    chassis.moveToPoint(-30, 47.9, 2000, {.forwards = false});
    Scraper.set_value(false);
    pros::delay(20);
    IntakedBalls = 0;
    Loader.move(127);
    */
    

    ///Left Auto With Distanse sensor stuff///
    
    BallTask.suspend();
    Scraper.set_value(true);
    chassis.setPose(-61.8, 16.7, 90);
    Loader.move(-70);
    chassis.moveToPoint(-38.7, 16.7, 2000);
    Scraper.set_value(false);
    chassis.turnToPoint(-22, 21.8, 2000);
    chassis.moveToPoint(-22, 21.8, 1500,{.maxSpeed = 35});
    LowerIntake.move(127);
    UpperIntake.move(127);
    chassis.turnToPoint(-15, 12, 2000, {.forwards = false});
    UpperIntake.move(0);
    chassis.moveToPoint(-15, 12, 2000, {.forwards = false});
    UpperIntake.move(127);
    Loader.move(50);
    pros::delay(2500);
    Loader.move(-20);
    chassis.turnToPoint(-47.4, 47, 2000);
    Scraper.set_value(true);
    chassis.moveToPoint(-47.4, 47, 2000);
    chassis.turnToPoint(-63, 47, 2000);
    chassis.moveToPoint(-63, 47, 2000);
    Lift.set_value(true);
    pros::delay(2500);
    chassis.moveToPoint(-37, 47.9, 2000, {.forwards = false});
    Scraper.set_value(false);
    pros::delay(20);
    IntakedBalls = 0;
    pros::delay(2000);
    //CheckGoal();
    Loader.move(127);

}



void opcontrol() {
    
      BallTask.resume();

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
            
		}else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            Scraper.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            Scraper.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            Lift.set_value(true);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            Lift.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            LowerIntake.move(-127);
            UpperIntake.move(-127);
            Loader.move(-127);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            DeScore.set_value(false);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            DeScore.set_value(true);
        }

        
        
        pros::delay(10);
    }
    
}