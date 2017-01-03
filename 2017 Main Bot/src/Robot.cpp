#include "WPILib.h"

class Robot: public IterativeRobot
{
	//Construct for objects here
	Joystick *driveStick;
	Joystick *manipulatorStick;
	CANTalon *lDrive1;
	CANTalon *lDrive2;
	CANTalon *rDrive1;
	CANTalon *rDrive2;
	DoubleSolenoid *Shifter;
	
public:
	Robot() {
		Wait(1);
		driveStick = new Joystick (0);
		manipulatorStick = new Joystick (1);
		lDrive1 = new CANTalon(1);
		lDrive2 = new CANTalon(2);
		rDrive1 = new CANTalon(3);
		rDrive2 = new CANTalon(4);
		Shifter = new DoubleSolenoid (0,1);

		//CameraServer::GetInstance()->SetQuality(50);
		//table = NetworkTable::GetTable("GRIP/myContoursReport");
	}

private:
	LiveWindow *lw = LiveWindow::GetInstance();
		const std::string autoNameDefault = "Default";
		const std::string autoNameCustom = "My Auto";
		std::string autoSelected;
		double leftX = 0;
		double leftY = 0;
		double rightX = 0;
		double rightY = 0;
		double mLeftY = 0;
		double mRightY = 0;
		double leftTrigger = 0;
		double rightTrigger = 0;
		double mRightTrigger = 0;
		double mLeftTrigger = 0;
		bool dUp = false;
		bool dRight = false;
		bool dDown = false;
		bool dLeft = false;
		bool dUpR = false;
		bool dUpL = false;
		bool dDownR = false;
		bool dDownL = false;
		bool autoShot = false;
		bool buttonVal0 = false;
		bool buttonVal1 = false;
		bool buttonVal2 = false;
		bool buttonVal3 = false;
		double threshold = 0.09;


	void AutonomousInit() // code for auto mode !!!this will run and stop only when disabled!!!
	{
		// code for auto goes here !!!It will run when auto is enabled on the ds!!!
	}

	void TeleopInit() // holds code for pre teleop enable
	{

	}

	void TeleopPeriodic() // code for the robot to move is placed here (runs when
	// the robot is enabled in teleop mode)
	{
		//CameraServer::GetInstance()->StartAutomaticCapture("cam0");
				//SmartDashboard::PutNumber("DB/Slider 0", accel->GetX());
				//SmartDashboard::PutNumber("DB/Slider 1", accel->GetY());
		//		SmartDashboard::PutNumber("DB/Slider 2", accel->GetZ());
				//SmartDashboard::PutNumber("DB/Slider 3", gyro->GetAngle());
				//frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

				//distance = (sonicIn->GetRaw() * inches);
				//sonicOut->Pulse(10);

				leftX = driveStick->GetRawAxis(0);
				if(fabs(leftX) < threshold)
					leftX = 0;
				leftY = driveStick->GetRawAxis(1);
				if(fabs(leftY) < threshold)
					leftY = 0;
				mLeftY = manipulatorStick->GetRawAxis(1);
				if(fabs(mLeftY) < threshold)
					mLeftY = 0;
				rightX = driveStick->GetRawAxis(4);
				if(fabs(rightX) < threshold)
					rightX = 0;
				rightY = driveStick->GetRawAxis(5);
				if(fabs(rightY) < threshold)
					rightY = 0;
				mRightY = manipulatorStick->GetRawAxis(5);
				if(fabs(mRightY) < threshold)
					mRightY = 0;
				leftTrigger = (driveStick->GetRawAxis(2));
				if(fabs(leftTrigger) < (threshold))
					leftTrigger = 0;
				rightTrigger = (driveStick->GetRawAxis(3));
				if(fabs(rightTrigger) < (threshold))
					rightTrigger = 0;
				mRightTrigger = (manipulatorStick->GetRawAxis(3));
				if(fabs(mRightTrigger) < (threshold))
					mRightTrigger = 0;
				mLeftTrigger = (manipulatorStick->GetRawAxis(2));
				if(fabs(mLeftTrigger) < (threshold))
					mLeftTrigger = 0;

				leftX = pow(fabs(leftX), 1.4);
				if(driveStick->GetRawAxis(0) > 0) {
					leftX = -leftX;
				}
				/*if(driveStick->GetRawAxis(1) < 0) {
					leftY = -leftY;
				}*/
				leftY = pow(fabs(leftY), 1.4);
				if(driveStick->GetRawAxis(1) < 0) {
					leftY = -leftY;
				}
				rightX = pow(fabs(rightX), 1.4);
				if(driveStick->GetRawAxis(4) > 0) {
					rightX = -rightX;
				}
				rightY = pow(fabs(rightY), 1.4);
				if(driveStick->GetRawAxis(5) > 0) {
					rightY = -rightY;
				}
				mRightY = pow(fabs(mRightY), 1.4);
				if(manipulatorStick->GetRawAxis(5) > 0) {
					mRightY = -mRightY;
				}
				mLeftY = pow(fabs(mLeftY), 1.4);
				if(manipulatorStick->GetRawAxis(1) > 0) {
					mLeftY = -mLeftY;
				}

				rDrive1->Set(-(leftY - leftX));
				rDrive2->Set(-(leftY - leftX));
				lDrive1->Set(leftY + leftX);
				lDrive2->Set(leftY + leftX);

				if(driveStick->GetRawButton(3)) {
					Shifter->Set(DoubleSolenoid::Value::kReverse);
					driveStick->SetRumble(driveStick->kLeftRumble, 1);
					driveStick->SetRumble(driveStick->kRightRumble, 1);
					Wait(0.01);
					driveStick->SetRumble(driveStick->kLeftRumble, 0);
					driveStick->SetRumble(driveStick->kRightRumble, 0);
					//rShifter->Set(DoubleSolenoid::Value::kForward);
				} else if(driveStick->GetRawButton(2)) {
					Shifter->Set(DoubleSolenoid::Value::kForward);
					driveStick->SetRumble(driveStick->kLeftRumble, 1);
					driveStick->SetRumble(driveStick->kRightRumble, 1);
					Wait(0.01);
					driveStick->SetRumble(driveStick->kLeftRumble, 0);
					driveStick->SetRumble(driveStick->kRightRumble, 0);
					//rShifter->Set(DoubleSolenoid::Value::kReverse);
				}

				Wait(0.005);
	}// end TeleopPeriodic

	void TestInit()
	{

	}

	void TestPeriodic()
	{

	}

};

START_ROBOT_CLASS(Robot)
