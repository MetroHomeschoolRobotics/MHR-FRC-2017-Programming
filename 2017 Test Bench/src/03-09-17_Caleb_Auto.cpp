#include "WPILib.h"
#include "CANTalon.h"
#include "Spark.h"

class Robot: public IterativeRobot
{

		//Construct for objects here
		Joystick *driveStick;
		Joystick *manipulatorStick;
		CANTalon *leftDrive1;
		CANTalon *leftDrive2;
		CANTalon *rightDrive1;
		CANTalon *rightDrive2;
		CANTalon *shooterTop;
		CANTalon *shooterBottom;
		Spark *climber;
		Spark *intake;
		Spark *lowerIntakeRoller;

	public:
		Robot()
		{
			Wait(1);
			driveStick = new Joystick(0);
			manipulatorStick = new Joystick(1);
			leftDrive1 = new CANTalon(1);
			leftDrive2 = new CANTalon(2);
			rightDrive1 = new CANTalon(3);
			rightDrive2 = new CANTalon(4);
			shooterTop = new CANTalon(5);
			shooterBottom = new CANTalon(6);
			climber = new Spark(0);
			intake = new Spark(1);
			lowerIntakeRoller = new Spark(2);
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
		double mLeftTrigger = 0;bool dUp = false;bool dRight = false;bool dDown =
		false;bool dLeft = false;bool dUpR = false;bool dUpL = false;bool dDownR =
		false;bool dDownL = false;bool autoShot = false;bool buttonVal0 =
		false;bool buttonVal1 = false;bool buttonVal2 = false;bool buttonVal3 =
		false;
		double threshold = 0.09;

		//vision tracking
		//vision tracking
		cs::UsbCamera cam1;
		int action = 1;
		double IMAGE_WIDTH = 320;
		double IMAGE_HEIGHT = 240;
		double COGX = 0;
		double COGY = 0;
		double LOuter = 0;
		double LInner = 0;
		double ROuter = 0;
		double RInner = 0;
		double shooterPos = 0;
		double shootCount = 0;
		double turnDistance = 0;
		double shootDistance = 0;
		double targetCenter = 75; // change me for the target center *note camera is offset
		double minTargetmargin = 6;
		double maxTargetmargin = -6;
		std::shared_ptr<NetworkTable> roboRealm;

		void RobotInit() override
		{
			roboRealm = NetworkTable::GetTable("cam1");

			cam1 = CameraServer::GetInstance()->StartAutomaticCapture("cam1",
					0);
			cam1.SetResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
			cam1.SetFPS(24);
		}

		void AutonomousInit() // code for auto mode !!!this will run and stop only when disabled!!!
		{
//			if (action == 1)
			if (targetCenter > ROuter)
			{
				SmartDashboard::PutString("Vision Says", "Turn Right!!!");
				leftDrive1->Set(-0.5);
				leftDrive2->Set(-0.5);
				rightDrive1->Set(-0.5);
				rightDrive2->Set(-0.5);
			}
			else if (targetCenter < LOuter)
			{
				SmartDashboard::PutString("Vision Says", "Turn Left!!!");
				leftDrive1->Set(-0.5);
				leftDrive2->Set(-0.5);
				rightDrive1->Set(-0.5);
				rightDrive2->Set(-0.5);
			}
			else if (targetCenter < ROuter && targetCenter > RInner)
			{
				SmartDashboard::PutString("Vision Says", "Turn Right");
				leftDrive1->Set(-0.25);
				leftDrive2->Set(-0.25);
				rightDrive1->Set(-0.25);
				rightDrive2->Set(-0.25);
			}
			else if (targetCenter < LOuter && targetCenter > LInner)
			{
				SmartDashboard::PutString("Vision Says", "Turn Left");
				leftDrive1->Set(0.25);
				leftDrive2->Set(0.25);
				rightDrive1->Set(0.25);
				rightDrive2->Set(0.25);
			}
			else if (targetCenter < RInner && targetCenter > LInner)
			{
				SmartDashboard::PutString("Vision Says", "Good!!!");
				leftDrive1->Set(.5);
				leftDrive2->Set(.5);
				rightDrive1->Set(.5);
				rightDrive2->Set(.5);
				Wait(5); //CHANGE THIS!!!!!!!!!!!

				leftDrive1->Set(0);
				leftDrive2->Set(0);
				rightDrive1->Set(0);
				rightDrive2->Set(0);
			}
			Wait(0.005);
			/*
			 if (action == 2)
			 {
			 if (targetCenter > ROuter)
			 {
			 SmartDashboard::PutString("Vision Says", "Turn Right!!!");
			 leftDrive1->Set(-0.5);
			 leftDrive2->Set(-0.5);
			 rightDrive1->Set(-0.5);
			 rightDrive2->Set(-0.5);
			 }
			 else if (targetCenter < LOuter)
			 {
			 SmartDashboard::PutString("Vision Says", "Turn Left!!!");
			 leftDrive1->Set(-0.5);
			 leftDrive2->Set(-0.5);
			 rightDrive1->Set(-0.5);
			 rightDrive2->Set(-0.5);
			 }
			 else if (targetCenter < ROuter && targetCenter > RInner)
			 {
			 SmartDashboard::PutString("Vision Says", "Turn Right");
			 leftDrive1->Set(-0.25);
			 leftDrive2->Set(-0.25);
			 rightDrive1->Set(-0.25);
			 rightDrive2->Set(-0.25);
			 }
			 else if (targetCenter < LOuter && targetCenter > LInner)
			 {
			 SmartDashboard::PutString("Vision Says", "Turn Left");
			 leftDrive1->Set(0.25);
			 leftDrive2->Set(0.25);
			 rightDrive1->Set(0.25);
			 rightDrive2->Set(0.25);
			 }
			 else if (targetCenter < RInner && targetCenter > LInner)
			 {
			 SmartDashboard::PutString("Vision Says", "Good!!!");
			 leftDrive1->Set(127);
			 leftDrive2->Set(127);
			 rightDrive1->Set(-127);
			 rightDrive2->Set(-127);
			 Wait(1000); //CHANGE THIS!!!!!!!!!!!

			 shooterTop->Set(127);
			 shooterBottom->Set(127);
			 Wait(5000);	//CHANGE THIS!!!!!!!!!!!

			 leftDrive1->Set(63.5);
			 leftDrive2->Set(63.5);
			 rightDrive1->Set(63.5);
			 rightDrive2->Set(63.5);
			 Wait(750); //CHANGE THIS!!!!!!!!!!!

			 leftDrive1->Set(127);
			 leftDrive2->Set(127);
			 rightDrive1->Set(-127);
			 rightDrive2->Set(-127);
			 Wait(1000); //CHANGE THIS!!!!!!!!!!!
			 }
			 Wait(0.005);
			 }*/
		}

		void TeleopInit() // holds code for pre teleop enable
		{

		}

		void TeleopPeriodic() // code for the robot to move is placed here (runs when
		// the robot is enabled in teleop mode)
		{
			//code for controller values

			leftX = driveStick->GetRawAxis(0);
			if (fabs(leftX) < threshold)
				leftX = 0;

			leftY = driveStick->GetRawAxis(1);
			if (fabs(leftY) < threshold)
				leftY = 0;

			mLeftY = manipulatorStick->GetRawAxis(1);
			if (fabs(mLeftY) < threshold)
				mLeftY = 0;

			rightX = driveStick->GetRawAxis(4);
			if (fabs(rightX) < threshold)
				rightX = 0;

			rightY = driveStick->GetRawAxis(5);
			if (fabs(rightY) < threshold)
				rightY = 0;

			mRightY = manipulatorStick->GetRawAxis(5);
			if (fabs(mRightY) < threshold)
				mRightY = 0;

			leftTrigger = (driveStick->GetRawAxis(2));
			if (fabs(leftTrigger) < (threshold))
				leftTrigger = 0;

			rightTrigger = (driveStick->GetRawAxis(3));
			if (fabs(rightTrigger) < (threshold))
				rightTrigger = 0;

			mRightTrigger = (manipulatorStick->GetRawAxis(3));
			if (fabs(mRightTrigger) < (threshold))
				mRightTrigger = 0;

			mLeftTrigger = (manipulatorStick->GetRawAxis(2));
			if (fabs(mLeftTrigger) < (threshold))
				mLeftTrigger = 0;

			leftX = pow(fabs(leftX), 1.4);
			if (driveStick->GetRawAxis(0) > 0)
			{
				leftX = -leftX;
			}

			leftY = pow(fabs(leftY), 1.4);
			if (driveStick->GetRawAxis(1) < 0)
			{
				leftY = -leftY;
			}

			rightX = pow(fabs(rightX), 1.4);
			if (driveStick->GetRawAxis(4) > 0)
			{
				rightX = -rightX;
			}

			rightY = pow(fabs(rightY), 1.4);
			if (driveStick->GetRawAxis(5) > 0)
			{
				rightY = -rightY;
			}

			mRightY = pow(fabs(mRightY), 1.4);
			if (manipulatorStick->GetRawAxis(5) > 0)
			{
				mRightY = -mRightY;
			}

			mLeftY = pow(fabs(mLeftY), 1.4);
			if (manipulatorStick->GetRawAxis(1) > 0)
			{
				mLeftY = -mLeftY;
			}
			// End of code for controller values

			// drive code
			leftDrive1->Set(-leftY - leftX);
			leftDrive2->Set(-leftY - leftX);
			rightDrive1->Set(leftY - leftX);
			rightDrive2->Set(leftY - leftX);
			// end of drive code

			// climb code start
			if (driveStick->GetRawAxis(2))
			{
				climber->Set(leftTrigger);
			}
			else if (driveStick->GetRawButton(5))
			{
				climber->Set(-0.5);
			}
			else
			{
				climber->Set(0);
			}
			// climb code end

			// intake code start

			if (driveStick->GetRawButton(3)) // score low boiler
			{
				intake->Set(1);
				lowerIntakeRoller->Set(-1);
			}
			else if (driveStick->GetRawButton(2)) // score low boiler
			{
				intake->Set(1);
				lowerIntakeRoller->Set(-1);
			}
			else if (driveStick->GetRawButton(4)) // intake to bot
			{
				intake->Set(1);
				lowerIntakeRoller->Set(1);
			}
			else if (driveStick->GetRawButton(1)) // its smuckers button
			{
				intake->Set(-1);
				lowerIntakeRoller->Set(-1);
			}
			else
			{
				intake->Set(0);
				lowerIntakeRoller->Set(0);
			}
			// end intake code
		}
};

START_ROBOT_CLASS(Robot);
