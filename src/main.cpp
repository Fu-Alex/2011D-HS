#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-14, -15, -16},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({11, 12, 13}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(18);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(17);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-2);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -4);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, .2);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              480, // drivetrain rpm is 360
                              10 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(4.6, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            5, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2.35, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
pros::MotorGroup intake({4,-7});
pros::Motor lift (-3); 
pros::adi::DigitalOut clamp ('a', false);
pros::adi::DigitalOut redirect ('b', false);
pros::adi::DigitalOut sweeper ('c', false);
pros::Rotation liftRot(-6);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
int liftpos =  0;
void liftPID(double target, int TO) {

  // Constants

  double kP = 5;

  double kI = 0.0;
  double kD = 10;
  double threshold = 2;


  float errorD = 0;
  double errorD2 = 0;
  double proportion = 0;
  double integralZone = fabs(fabs(target) - fabs(liftRot.get_position()/1000)) * 0.25;
  double integral = 0;
  double integralLimit = 10;
  double lastErrorD;
  double derivative;
  double prevError = 0;
  int direction = 1;
  float liftVelo;


  while (true) {

      errorD = (fabs(target - fabs(liftRot.get_position()))/1000);
      errorD2 = (target) - liftRot.get_position();
      proportion = errorD;
      direction = errorD2/fabs(errorD2);
      if (errorD > threshold){
          if(errorD2 >   0){
            liftVelo = 127;
          }
          else{
            liftVelo = -127;
          }
      }
      else{
      if(fabs(errorD) < integralZone){
        integral = integral + errorD;
      }
      else{
        integral = 0;
      }
      if(integral > integralLimit){
        integral = integralLimit;
      }

      derivative = errorD - lastErrorD;
      lastErrorD = errorD;
      if(errorD < .5  && errorD > -.5){
        proportion = 0;
        derivative = 0;
      }

      liftVelo = (kP*proportion + kI*integral + kD*derivative);

      liftVelo = liftVelo * direction;
      }
    if(target<1000 && errorD <.5){
        liftVelo = 0;
      }
      if(liftVelo > 127){
        liftVelo = 127;
      }
      if (liftVelo < -127){
        liftVelo = -127;
      }
      //pros::lcd::print(5, "Error: %f", errorD);
     // pros::lcd::print(6, "Error: %f", liftVelo);
      lift.move(liftVelo);
      pros::lcd::print(3, "pos: %f",errorD);// heading
      delay(TO);
  }

}

void set_lift_pos(int pos, int timeouts) {

  static std::unique_ptr<pros::Task> pidTask {};

  if (pidTask != nullptr) { pidTask->remove(); }

  pidTask = (pos == -1) ? nullptr : std::make_unique<pros::Task>([=]{ liftPID(pos,timeouts); });

}
int height = 0;
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // calibrate the balls
    liftRot.set_position(0);
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(4, "pos: %i",height);
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources

            pros::delay(30);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

    // get a path used for pure pursuit
// this needs to be put outside a function
 // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void safeRightRed(){
chassis.moveToPoint(12,-30,1200,{.forwards = false, .maxSpeed = 70});
redirect.set_value(true);
intake.move(127);
chassis.waitUntilDone();
clamp.set_value(true);
delay(100);
chassis.moveToPose(-12,-30,90,1000,{.maxSpeed = 60});
delay(300);
redirect.set_value(false);
chassis.moveToPoint(0,-30,500,{.forwards = false});
chassis.moveToPoint(-24,-12,800,{.maxSpeed = 90, .minSpeed = 70});
sweeper.set_value(true);
chassis.moveToPoint(-30,7,800,{.maxSpeed = 70});
set_lift_pos(6000,10);
chassis.turnToPoint(0,0,800, {.maxSpeed = 80});
chassis.turnToPoint(-30,11,800);
sweeper.set_value(false);
chassis.moveToPoint(-30,13,1000,{.maxSpeed = 70});
chassis.waitUntilDone();
delay(250);
chassis.moveToPoint(-20,-12,800,{.forwards = false});
chassis.moveToPoint(12,-18,800, {.forwards = false});
// chassis.turnToPoint(28,8,800);//elims
// intake.move(127);
// chassis.moveToPoint(29,8,1000);------------------------------
chassis.turnToPoint(27,-18.6,800);
chassis.moveToPoint(27,-18.6,800);
sweeper.set_value(true);
chassis.turnToPoint(27,6,800);//quals
intake.move(127);
chassis.moveToPoint(29,7,1300);//-------------------------------
sweeper.set_value(false);
chassis.turnToHeading(30,800);
chassis.waitUntilDone();
set_lift_pos(2000,10);
delay(300);
//chassis.moveToPoint(26,-28,800,{.forwards = false});
chassis.turnToPoint(-30,10,800,{.forwards = false});//use for elims
chassis.moveToPoint(-30,10,800, {.forwards = false});
//chassis.waitUntilDone();
//chassis.turnToHeading(110,800);

}
void safeBlueLeft(){
chassis.moveToPoint(-12,-30,1200,{.forwards = false, .maxSpeed = 70});
redirect.set_value(true);
intake.move(127);
chassis.waitUntilDone();
clamp.set_value(true);
chassis.moveToPose(14,-30,90,1000,{.maxSpeed = 70});
delay(300);
redirect.set_value(false);
set_lift_pos(6000,10);
chassis.turnToPoint(-27,-18.6,800);
chassis.moveToPoint(-27,-18.6,800);
sweeper.set_value(true);
chassis.turnToPoint(-27,6,800);//quals
intake.move(127);
chassis.moveToPoint(-27,6,1000);//-------------------------------
sweeper.set_value(false);
chassis.turnToHeading(-42,800);
chassis.waitUntilDone();
set_lift_pos(0,10);
delay(300);
chassis.moveToPoint(0,-0,800,{.forwards = false});
chassis.turnToPoint(15,6,700);
intake.move(127);
set_lift_pos(60001111,10);
chassis.moveToPoint(17,8,1200);
sweeper.set_value(true);
chassis.turnToHeading(180,400);
chassis.turnToPoint(28,14,800);
sweeper.set_value(false);
chassis.moveToPoint(31,17,800);
chassis.waitUntilDone();
chassis.moveToPoint(-24,-36,1400,{.forwards = false});


}
void redLeft(){
chassis.moveToPoint(-10,-27,1200,{.forwards = false, .maxSpeed = 70});
chassis.waitUntilDone();
clamp.set_value(true);
intake.move(127);
delay(200);
chassis.turnToPoint(-4,-38,600);
chassis.moveToPoint(-4,-38,800,{.minSpeed = 65});
redirect.set_value(false);
chassis.moveToPoint(10,-45,500, {.maxSpeed = 65});
chassis.moveToPoint(23,-45,500, {.maxSpeed = 65});
delay(200);
set_lift_pos(6000,10);
chassis.turnToPoint(12,-35,700);
set_lift_pos(0,10);
chassis.moveToPoint(12,-35,800);
delay(250);
redirect.set_value(true);
chassis.turnToPoint(-20,-28,500);
chassis.moveToPoint(-20,-28,700,{.minSpeed = 70});
chassis.moveToPoint(-29,-18,800);
sweeper.set_value(true);
delay(400);
set_lift_pos(6000,10);
redirect.set_value(false);
chassis.turnToHeading(0,700);
chassis.moveToPoint(-31,7,800);
sweeper.set_value(false);
chassis.turnToHeading(-44,700);
chassis.waitUntilDone();
set_lift_pos(2000,10);
delay(250);
intake.move(-127);
chassis.moveToPoint(-25,-5,700,{.forwards = false});
chassis.turnToPoint(15,6,700);
intake.move(127);
set_lift_pos(6000,10);
chassis.moveToPoint(17,9,1200);
sweeper.set_value(true);
chassis.turnToHeading(180,400);
chassis.turnToPoint(28,14,800);
sweeper.set_value(false);
chassis.moveToPoint(31,17,800);
delay(250);
//chassis.moveToPoint(-24,0,1500,{.forwards =  false});
chassis.moveToPoint(-28,-44,1500,{.forwards =  false});

}
void blueRight(){
chassis.moveToPoint(10,-27,1200,{.forwards = false, .maxSpeed = 70});
chassis.waitUntilDone();
clamp.set_value(true);
intake.move(127);
delay(200);
chassis.turnToPoint(4,-38,600);
chassis.moveToPoint(4,-38,800,{.minSpeed = 65});
redirect.set_value(false);
chassis.moveToPoint(-10,-45,500, {.maxSpeed = 65});
chassis.moveToPoint(-23,-45,500, {.maxSpeed = 65});
delay(200);
set_lift_pos(8000,10);
chassis.turnToPoint(-8,-31,800);
chassis.waitUntilDone();
set_lift_pos(0,10);

chassis.moveToPoint(-8,-31,800,{.maxSpeed = 70});
delay(250);
redirect.set_value(true);
chassis.moveToPoint(-12,-35,500,{.forwards = false,.maxSpeed = 70});
chassis.moveToPoint(-24,-12,800,{.maxSpeed = 90, .minSpeed = 70});
sweeper.set_value(true);
chassis.moveToPoint(-28,10,800);
delay(250);
set_lift_pos(6000,10);
chassis.turnToPoint(0,0,600);
chassis.turnToPoint(-30,11,500);
sweeper.set_value(false);
redirect.set_value(false);
chassis.moveToPoint(-30,13,800,{.maxSpeed = 70});
chassis.waitUntilDone(); 
chassis.moveToPoint(-12,-24,900,{.forwards = false});
chassis.turnToPoint(25,8,700);
chassis.moveToPoint(25,8,1200);
intake.move(-127);
chassis.waitUntilDone();
chassis.turnToHeading(35,800);
chassis.waitUntilDone();
set_lift_pos(0,10);
delay(300);
chassis.moveToPoint(24,0,800,{.forwards = false});
chassis.turnToPoint(30,-24,800);
intake.move(127);
sweeper.set_value(true);
chassis.moveToPoint(30,-24,800);
}
void skills(){  
    chassis.setPose(0,51.5,0);
    //score on alliance stake
    intake.move(127);
    redirect.set_value(true);
    delay(700);
    set_lift_pos(6000,10);
    delay(400);
    chassis.moveToPoint(0,59,700);
    chassis.waitUntilDone();
    set_lift_pos(2000,10);
    delay(300);
    redirect.set_value(false);
    //pick up first goal
    chassis.moveToPoint(0,48,900,{.forwards = false});
    chassis.turnToPoint(48,48,700,{.forwards =false});
    chassis.moveToPoint(24,48,1200, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    set_lift_pos(8000,10);
    clamp.set_value(true);
    delay(200);
    //intake 6 rings
    chassis.turnToPoint(48,48,700);
    chassis.moveToPoint(60,46,1100, {.maxSpeed   = 75});
    chassis.waitUntilDone();
    chassis.turnToPoint(48,60,700);
    chassis.moveToPoint(48,60,1000, {.maxSpeed = 60, .minSpeed = 10});
    chassis.moveToPoint(24,24,1500, {.maxSpeed = 80});
    chassis.turnToPoint(48,24,650);
    chassis.moveToPoint(44,24,1000,{.maxSpeed = 70, .minSpeed = 30});
    chassis.moveToPoint(58,-5,1300);
    chassis.moveToPoint(63,60,1300,{.forwards = false});
    chassis.waitUntilDone();
    //drop goal #1 in corner
    clamp.set_value(false);
    delay(250);
    chassis.moveToPoint(48,48,1000);
    chassis.turnToPoint(-48,48,1000, {.forwards = false});
    chassis.moveToPose(0,48,3000,90,{.forwards = false, .minSpeed = 70});
    //pick up goal #2
    chassis.moveToPoint(-28,48,3000,{.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    clamp.set_value(true);
    delay(200);
    //intake 6 rings
    chassis.turnToPoint(-24,24,700);
    chassis.moveToPoint(-24,24,750,{.maxSpeed = 60, .minSpeed = 20});
    chassis.moveToPoint(-58,-3,1500,{.maxSpeed = 80});
    chassis.waitUntilDone();
    delay(200);
    chassis.turnToPoint(-48,24,750);
    chassis.moveToPoint(-48,24,750,{.maxSpeed = 70, .minSpeed = 60});
    chassis.moveToPoint(-48,63,2000,{.maxSpeed = 65});
    chassis.moveToPoint(-40,48,750, {.forwards = false});
    chassis.turnToPoint(-60,48,700);
    chassis.moveToPoint(-60,48,700);
    chassis.turnToPoint(-66,70,750, {.forwards = false});
    chassis.moveToPoint(-66,57,800,{.forwards = false});
    //drop second  goal

    //grab ring and put into redirect
    set_lift_pos(0,10);
    // I <3 Alex Fu
    chassis.moveToPoint(-48,-50,1800, {.maxSpeed = 90});
    clamp.set_value(false);
    redirect.set_value(true);
    chassis.waitUntilDone();
    delay(500);
    // //sweaty dude balls
    chassis.moveToPoint(-48,-1.5,1500,{.forwards = false, .maxSpeed = 70});
    // chassis.waitUntilDone();
    // //score on right neutral wall stake
    chassis.turnToPoint(-72,-1.5,750);
    chassis.waitUntilDone();
    set_lift_pos(8300,10);
    delay(650);
   
}   
void skills2(){
     chassis.moveToPoint(-70,-1.5,750);
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x-2, 0, imu.get_heading());
    set_lift_pos(0,10);
    delay(500);
    // //grab 2 more rings and go to neutral stake on left side
    chassis.moveToPoint(-48,-1,700,{.forwards = false});
    chassis.turnToPoint(-24,-22,800);
    chassis.moveToPoint(-24,-22,800, {.maxSpeed = 90});
    chassis.turnToPoint(0,0,600);
    chassis.moveToPoint(0,0,1000, {.maxSpeed = 70});
    chassis.moveToPoint(24,24,1000,{.maxSpeed = 60});
    chassis.turnToPoint(48,4,800);
    // //score on left neutral wall stake
    chassis.moveToPoint(48,4,1000);
    set_lift_pos(8300,10);
    chassis.turnToPoint(72,4,700);
    chassis.moveToPoint(70,4,700);
    chassis.waitUntilDone();
    set_lift_pos(4000,10);
    delay(500);
    // //intake another ring into redirect
    chassis.moveToPoint(48,2,800,{.forwards = false});
    set_lift_pos(0,0);
     chassis.turnToPoint(48,-24,800);
     chassis.moveToPoint(48,-27,1000);
    // chassis.waitUntilDone();
    // //grab goal #3
    chassis.waitUntilDone();
     delay(500);
     chassis.turnToPoint(24,-45,1000,{.forwards = false});
     chassis.moveToPoint(24,-45,800, {.forwards=false});
     chassis.turnToPoint(0,-45,800,{.forwards  = false});
     chassis.moveToPoint(-5,-47,1200,{.forwards = false, .maxSpeed = 70});
     set_lift_pos(0,10);
     chassis.waitUntilDone();
     clamp.set_value(true);
     delay(200);
     intake.move(127);
     //score on blue wall stake
     set_lift_pos(6000,10);
    chassis.moveToPoint(2,-45,800);
     chassis.turnToPoint(2,-100, 800);
     chassis.moveToPoint(2,-58,1000);
     chassis.waitUntilDone();
     chassis.turnToHeading(185,700);
     chassis.waitUntilDone();
     set_lift_pos(2000,10);
     delay(300);
    chassis.moveToPoint(0,-48,800, {.forwards  =  false});
    chassis.turnToPoint(60,-48,800);
    set_lift_pos(8000,10);
    // //intake rings
    redirect.set_value(false);
    chassis.moveToPoint(44,-48,800, {.minSpeed = 60});
    chassis.moveToPoint(65,-48,500,{.maxSpeed = 60});
    chassis.turnToPoint(65,-65,800,{.forwards = false});
    // //drop goal in corner
    // chassis.turnToPoint(68,-68,800, {.forwards = false});
    chassis.moveToPoint(62,-57,800,{.forwards = false});
    // // chassis.turnToPoint(48,-48,800);
    // // chassis.moveToPoint(48,-48,800);
    // // chassis.turnToPoint(72,-48,800);
    // // chassis.moveToPoint(65,-48,800);
    // // chassis.turnToPoint(65,-65,800, {.forwards = false});
    // // chassis.moveToPoint(65,-65,800,{.forwards = false});
    chassis.waitUntilDone();
    clamp.set_value(false);
    intake.move(0);
    // // chassis.waitUntilDone();
    // //push goal #4 into opposite corner
    chassis.moveToPoint(-60,-70,10000);

}
void autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    chassis.setPose(0,0,0);
     //skills();
    //skills2();
    safeRightRed();
    //balls
}

/**
 * Runs in driver control
 */

bool redirecting = false;
bool clamped = false;
bool sweep = false;
void opcontrol() {
    // controller pepe popo
    // loop to continuously update motors
    pros::Task liftcontrol ( [] { liftPID(0,1000); } );
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightX);
        chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
        lift.set_brake_mode(MOTOR_BRAKE_COAST);
        //intake meow
            if(controller.get_digital(DIGITAL_R1)){
                intake.move(127);
                if(controller.get_digital(DIGITAL_L1)){
                    redirecting = true;
                }
                else{
                    redirecting=false;
                }
            }
            else if(controller.get_digital(DIGITAL_R2)){
                intake.move(-127);
            }
            else{
                intake.move(0);
            }
// meow for me kitten

            //lift
            if(controller.get_digital_new_press(DIGITAL_L1)&&!controller.get_digital(DIGITAL_R1)){
                height = height + 1;
                if(height > 2){
                    height = 1;
                }
            }
            else if(controller.get_digital(DIGITAL_L2)){
                set_lift_pos(3000,1000);
                height = 0;
            }

            if  (height == 0&&!controller.get_digital(DIGITAL_L2)){
                set_lift_pos(0,1000);
            }
            else if(height == 1){
                set_lift_pos(8000,1000);
            }
            else if(height == 2){
                set_lift_pos(6000,1000);
            }
            //back clamp
        if(controller.get_digital_new_press(DIGITAL_B)){
            clamped = !clamped;
        }
        if(controller.get_digital_new_press(DIGITAL_A)){
            sweep = !sweep;
        }
        clamp.set_value(clamped);
        redirect.set_value(redirecting);
        sweeper.set_value(sweep);
        // delay to save resources
        pros::delay(10);
    }
}