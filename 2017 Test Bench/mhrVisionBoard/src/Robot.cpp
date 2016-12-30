#include "WPILib.h"

class Robot: public IterativeRobot
{
	//Construct for objects here
	Joystick *driveStick;
	Servo *leftServo = new Servo(1);
	Servo *rightServo = new Servo(2);

	ADXRS450_Gyro *gyro;
	ADXL362 *accel;

public:
	Robot() {
		Wait(1);
		driveStick = new Joystick (0);
		leftServo = new Servo(1);
		rightServo = new Servo(2);

		gyro = new ADXRS450_Gyro();
		accel = new ADXL362();


		//CameraServer::GetInstance()->SetQuality(50);
		//table = NetworkTable::GetTable("GRIP/myContoursReport");
	}

private:
	double targetCenter = 150; // change me for the target center *note camera is offset
	double minTargetmargin = 6;
	double maxTargetmargin = -6;

	void AutonomousInit() // code for auto mode !!!this will run and stop only when disabled!!!
	{

	}

	void TeleopInit() // holds code for pre teleop enable
	{
		leftServo->Set(.5);
		rightServo->Set(.5);
	}

	void TeleopPeriodic() // code for the robot to move is placed here (runs when
	// the robot is enabled in teleop mode)
	{
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");

		if (driveStick->GetRawButton(7))
		{
			leftServo->SetAngle(75);
		}

		if (driveStick->GetRawButton(8))
		{
			rightServo->SetAngle(75);
		}
	}// end TeleopPeriodic

	void TestInit()
	{

	}

	void TestPeriodic()
	{

	}

};

START_ROBOT_CLASS(Robot)
