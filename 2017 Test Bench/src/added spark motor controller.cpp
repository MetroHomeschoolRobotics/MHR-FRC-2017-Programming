#include "WPILib.h"
#include "Spark.h"

class Robot: public IterativeRobot
{
	//Construct for objects here
	Joystick *driveStick;
	Servo *leftServo = new Servo(1);
	Servo *rightServo = new Servo(2);
	Spark *spark1;



	//ADXRS450_Gyro *gyro;
	//ADXL362 *accel;

public:
	Robot() {
		Wait(1);
		driveStick = new Joystick (0);
		leftServo = new Servo(1);
		rightServo = new Servo(2);
		spark1 = new Spark(0);

		//gyro = new ADXRS450_Gyro();
		//accel = new ADXL362();


		//CameraServer::GetInstance()->SetQuality(50);
		//table = NetworkTable::GetTable("GRIP/myContoursReport");
	}

private:
	//double targetCenter = 150; // change me for the target center *note camera is offset
	//double minTargetmargin = 6;
	//double maxTargetmargin = -6;

	void AutonomousInit() // code for auto mode !!!this will run and stop only when disabled!!!
	{
		// code for auto goes here !!!It will run when auto is enabled on the ds!!!
	}

	void TeleopInit() // holds code for pre teleop enable
	{
		leftServo->Set(.5);
		rightServo->Set(.5);
	}

	void TeleopPeriodic() // code for the robot to move is placed here (runs when
	// the robot is enabled in teleop mode)
	{
		//CameraServer::GetInstance()->StartAutomaticCapture("cam0");

		if (driveStick->GetRawButton(3)) // button 3 is the X button on an xpad
		{
			leftServo->SetAngle(75);
		}

		if (driveStick->GetRawButton(2)) // button 2 is the B button on an xpad
		{
			rightServo->SetAngle(75);
		}

		if (driveStick->GetRawButton(4))
		{
			spark1->Set(1);
		}
		else if (driveStick->GetRawButton(1))
		{
			spark1->Set(-1);
		}

	}// end TeleopPeriodic

	void TestInit()
	{
		// we dont use this section
	}

	void TestPeriodic()
	{
		// this is test mode on the ds we our confident in our release versions XD we dont use this
	}

};

START_ROBOT_CLASS(Robot)
