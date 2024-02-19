#include "main.h"
#include "lemlib/api.hpp"

// drive motors
pros::Motor lF(1, pros::E_MOTOR_GEARSET_06);   // left front motor. port 12
pros::Motor lM(11, pros::E_MOTOR_GEARSET_06);  // left middle motor. port 11
pros::Motor lB(-13, pros::E_MOTOR_GEARSET_06); // left back motor. port 13, reversed
pros::Motor rF(-14, pros::E_MOTOR_GEARSET_06); // right front motor. port 14, reverse
pros::Motor rM(-16, pros::E_MOTOR_GEARSET_06); // right middle motor. port 16, reversed
pros::Motor rB(15, pros::E_MOTOR_GEARSET_06);  // right back motor. port 15
pros::Motor smallPuncher(18, pros::E_MOTOR_GEAR_200);
pros::Motor bigPuncher(12, pros::E_MOTOR_GEAR_200, true);
pros::Motor intake(3, pros::E_MOTOR_GEAR_200);

// solenoids
pros::ADIDigitalOut singleLeftWing('C');
pros::ADIDigitalOut singleRightWing('D');
pros::ADIDigitalOut doubleHang('G');

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB});	// left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group
pros::MotorGroup puncher({smallPuncher, bigPuncher});

// Inertial Sensor on port
pros::Imu inertial_sensor(4); // port 4

lemlib::Drivetrain_t drivetrain{
	&leftMotors,  // left drivetrain motors
	&rightMotors, // right drivetrain motors
	10.75,		  // track width
	3.25,		  // wheel diameter
	400			  // wheel rpm
};

// left tracking wheel encoder
pros::ADIEncoder left_enc('E', 'F', true); // ports A and B, reversed

// back tracking wheel encoder
pros::ADIEncoder back_enc('A', 'B', true); // ports C and D, not reversed

// left tracking wheel
lemlib::TrackingWheel left_tracking_wheel(&left_enc, 2.75, -0.3); // 2.75" wheel diameter, -4.6" offset from tracking center
// back tracking wheel
lemlib::TrackingWheel back_tracking_wheel(&back_enc, 2.75, -3.4); // 2.75" wheel diameter, -4.5" offset from tracking center

// odometry struct
lemlib::OdomSensors_t sensors{
	&left_tracking_wheel, // vertical tracking wheel 1
	nullptr,			  // vertical tracking wheel 2
	&back_tracking_wheel, // horizontal tracking wheel 1
	nullptr,			  // we don't have a second tracking wheel, so we set it to nullptr
	&inertial_sensor	  // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController{
	25,	 // kP
	175, // kD
	1,	 // smallErrorRange
	100, // smallErrorTimeout
	3,	 // largeErrorRaxnge
	500, // largeErrorTimeoutsa
	5	 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController{
	4,	 // kP
	32,	 // kD
	1,	 // smallErrorRange
	100, // smallErrorTimeout
	3,	 // largeErrorRange
	500, // largeErrorTimeout
	0	 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void shove(int time, bool direction)
{
	if (direction == true)
	{
		leftMotors.move_velocity(600);
		rightMotors.move_velocity(600);
		pros::delay(time);
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);
	}
	if (direction == false)
	{
		leftMotors.move_velocity(-600);
		rightMotors.move_velocity(-600);
		pros::delay(time);
		leftMotors.move_velocity(0);
		rightMotors.move_velocity(0);
	}
}

void moveDrive(double leftY, double rightX)
{
	double kP = 0.9;
	double kT;
	if (fabs(leftY) < 10)
	{
		leftY = 0;
	}
	if (fabs(rightX) < 10)
	{
		rightX = 0;
	}
	if (rightX < 0)
	{
		kT = -pow(fabs(rightX) / 127.0, 3) * 52;
	}
	else
	{
		kT = pow(fabs(rightX) / 127.0, 3) * 52;
	}

	leftMotors.move_velocity(6 * (leftY / 1.27 * kP + kT));
	rightMotors.move_velocity(6 * (leftY / 1.27 * kP - kT));
}

void shootPuncher(bool puncherState)
{
	if (puncherState)
	{
		puncher.move_velocity(134.169);
	}
	else
	{
		puncher.move_velocity(0);
	}
}

void setWings(bool leftWingBool, bool rightWingBool)
{
	if (leftWingBool == true)
	{
		singleLeftWing.set_value(true);
	}
	else
	{
		singleLeftWing.set_value(false);
	}
	if (rightWingBool == true)
	{
		singleRightWing.set_value(true);
	}
	else
	{
		singleRightWing.set_value(false);
	}
}

void screen()
{
	// loop forever
	while (true)
	{
		lemlib::Pose pose = chassis.getPose();			// get the current position of the robot
		pros::lcd::print(0, "x: %f", pose.x);			// print the x position
		pros::lcd::print(1, "y: %f", pose.y);			// print the y position
		pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
		pros::delay(10);
	}
}

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	leftMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	chassis.calibrate();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void turnLeft(int destination) {
	leftMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	while(inertial_sensor.get_rotation() > destination) {
		leftMotors.move(-80);
		rightMotors.move(80);
	}
	leftMotors.move(0);
	rightMotors.move(0);
}
void sixball () {
	chassis.setPose(15.5, -59, -90);
	intake.move(127);
	chassis.moveTo(6.781, -59.073, 5000);
	pros::delay(50);
	chassis.moveTo(38, -58, 5000);
	setWings(true, false);
	chassis.moveTo(58.5, -50, 5000);
	turnLeft(-210);
	setWings(false, false);
	chassis.turnTo(59.635, -29.272, 2000);
	shove(600,true);
	// chassis.turnTo(41.301, -36, 2000);
	// chassis.moveTo(41.301, -36, 2000);
	// intake.move(-127);
	// pros::delay(100);
	// intake.move(127);
	// chassis.turnTo(8,-25,1000,false,75);
	// chassis.moveTo(9, -28, 1500);
	// chassis.turnTo(31, -19, 2000);
	// intake.move(-127);
	// pros::delay(250);
	// intake.move(127); 
	// chassis.turnTo(11.5, -4, 2000);
	// chassis.moveTo(11.56, -3.97, 5000);
	// setWings(true, true);
	// chassis.turnTo(42, -4, 2000);
	// shove(800,true);

}
void fiveball () {
	chassis.setPose(15.5, -59, -90);
	intake.move(127);
	chassis.moveTo(6.781, -59.073, 1500);
	pros::delay(50);
	chassis.moveTo(38, -58, 1500);
	chassis.turnTo(59, -41, 800, false, 100);
	chassis.moveTo(59, -41, 800);
	chassis.turnTo(59, -24, 800, false, 100);
	shove(500,true);
	shove(300, false);
	chassis.turnTo(28, -31, 1000, false,100);
	chassis.moveTo(28, -31, 1500);
	intake.move(-127);
	pros::delay(150);
	intake.move(127);
	chassis.turnTo(4, -15, 1000, false, 100);
	chassis.moveTo(4, -15, 1000);
	pros::delay(100);
	chassis.turnTo(44, -3, 1000, false, 100);
	chassis.moveTo(8, -20, 1000);
	intake.move(-127);
	pros::delay(250);
	intake.move(127);
	chassis.turnTo(6, 0, 1000, false, 100);
	chassis.moveTo(3, -1 , 1500);
	pros::delay(150);
	intake.move(0);
	chassis.turnTo(46, 0, 1500, false, 75);
	setWings(true, true);
	shove(800, true);
}



void teamwork_no()
{
	// Teamwork Auton no winpoint
	chassis.setPose(35, -58, 0);
	chassis.turnTo(58, -31, 1000, false, 75);
	chassis.moveTo(58, -31, 1000);
	chassis.turnTo(58, -21, 1000, false, 75);
	shove(300, true);
	shove(300, false);
	chassis.turnTo(28, -31, 1000, false, 75);
	chassis.moveTo(28, -31, 1500);
	intake.move(127);
	chassis.turnTo(4, -15, 1000, false, 75);
	chassis.moveTo(4, -15, 1000);
	pros::delay(150);
	intake.move(0);
	chassis.turnTo(44, -3, 1000, false, 75);
	chassis.moveTo(8, -23, 1000);
	intake.move(-127);
	pros::delay(400);
	intake.move(0);
	intake.move(127);
	chassis.turnTo(6, 0, 1000, false, 75);
	chassis.moveTo(4, 0, 1500);
	pros::delay(150);
	intake.move(0);
	chassis.turnTo(46, 0, 1500, false, 75);
	setWings(true, true);
	shove(800, true);
}

void teamwork_win(){
	chassis.setPose(-47,-54,0);
	chassis.turnTo(47,-7, 1000, false, 75);
	shootPuncher(true);
	pros::delay(1000);
	shootPuncher(false);
	setWings(true,false);
	chassis.turnTo(-63,-25,1500,false,80);
	chassis.turnTo(-55,-70, 1500,false,80);
	setWings(false,true);
	chassis.turnTo(-3,-62, 1500,true,80);
	chassis.moveTo(-9,-62,5000, 50);
	chassis.turnTo(-46, -7, 1500, false, 70);
}

void skillsAuton()
{
	chassis.setPose(35, 65, 0);
	chassis.turnTo(45, 47, 1000, true, 75);
	chassis.moveTo(59, 32, 1500);
	chassis.turnTo(59, 56, 1000, false, 75);
	chassis.moveTo(59, 55, 1500);
	chassis.turnTo(-46, 0, 1500, false, 75);
	shootPuncher(true);
	pros::delay(25000);
	shootPuncher(false);
	chassis.turnTo(35, 65, 1500, false, 75);
	chassis.moveTo(35, 65, 1000);
	chassis.turnTo(-35, 65, 1000, false, 75);
	chassis.moveTo(-35, 65, 3500);
	chassis.turnTo(-58, 26, 1000, true, 75);
	shove(1000, false);
	shove(450, true);
	shove(1000, false);
	shove(450, true);
	chassis.turnTo(-35, 39, 1000, false, 75);
	chassis.moveTo(-35, 39, 1500);
	chassis.turnTo(-12, 12, 1000, false, 75);
	chassis.moveTo(-12, 12, 1000);
	chassis.turnTo(-45, 3, 1000, false, 75);
	setWings(true, true);
	shove(750, true);
	shove(750, false);
	shove(750, true);
	shove(750, false);
	setWings(false, false);
	chassis.turnTo(-12, -25, 1500, false, 75);
	chassis.moveTo(-12, -25, 2000);
	chassis.turnTo(-45, 0, 1500, false, 75);
	setWings(true, true);
	shove(1000, true);
	shove(500, false);
}
void autonomous()
{
	// teamwork_no();
	//skillsAuton();
	fiveball();
	//teamwork_win();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * 
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	leftMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	bool hangState = false;
	bool hangPressed = false;
	bool puncherState = false;
	bool puncherPressed = false;

	while (true)
	{
		moveDrive(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X));


		if (master.get_digital(DIGITAL_R2))
		{
			if (!puncherPressed)
			{
				if (puncherState)
				{
					puncherState = false;
				}
				else if (!puncherState)
				{
					puncherState = true;
				}
				puncherPressed = true;
			}
		}
		else
		{
			puncherPressed = false;
		}

		if (puncherState)
		{
			shootPuncher(true);
		}
		else
		{
			shootPuncher(false);
		}

		if (master.get_digital(DIGITAL_L1))

		{
			intake.move(127);
		}
		else if (master.get_digital(DIGITAL_L2))
		{
			intake.move(-127);
		}
		else
		{
			intake.move(0);
		}

		if (master.get_digital(DIGITAL_X))
		{
			setWings(true, true);
		}
		else if (master.get_digital(DIGITAL_A))
		{
			setWings(false, true);
		}
		else if (master.get_digital(DIGITAL_Y))
		{
			setWings(true, false);
		}
		else if (master.get_digital(DIGITAL_B))
		{
			setWings(false, false);
		}

		if (master.get_digital(DIGITAL_R1))
		{
			if (!hangPressed)
			{
				if (hangState)
				{
					hangState = false;
				}
				else if (!hangState)
				{
					hangState = true;
				}
				hangPressed = true;
			}
		}
		else
		{
			hangPressed = false;
		}

		if (hangState)
		{
			doubleHang.set_value(true);
		}
		else
		{
			doubleHang.set_value(false);
		}

		pros::delay(20);
	}
}
