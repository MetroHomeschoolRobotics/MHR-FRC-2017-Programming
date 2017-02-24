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
		Spark *agitator;

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
			agitator = new Spark(2);
		}

	private:
		LiveWindow *lw = LiveWindow::GetInstance();
		const std::string autoNameDefault = "Default";
		const std::string autoNameCustom = "My Auto";
		std::string autoSelected;

		bool dUp = false;bool dRight = false;bool dDown =
		false;bool dLeft = false;bool dUpR = false;bool dUpL = false;bool dDownR =
		false;bool dDownL = false;bool autoShot = false;bool buttonVal0 =
		false;bool buttonVal1 = false;bool buttonVal2 = false;bool buttonVal3 =
		false;

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
		double threshold = 0.09;

		int trajectory = ((shootDistance * tan(45))
				- ((385.82698) * (shootDistance * shootDistance)
						/ (2 * 97.19685036 * cos(45) * cos(45))));
		double trajectoryCal = (fabs(sqrt(trajectory)));

		//vision tracking
		std::shared_ptr<NetworkTable> roboRealm;
		cs::UsbCamera cam1;
		cs::CvSink highGoalVid;
		int action = 1;
		double IMAGE_WIDTH = 320;
		double IMAGE_HEIGHT = 240;
		double COGX = 0;
		double COGY = 0;
		double LOuter = 0;
		double LInner = 0;
		double ROuter = 0;
		double RInner = 0;
		double centerPos = (ROuter + LOuter) / 2;
//		double centerPos2 = (LInner + RInner) / 2;
		double shootDistance = 0;
		int camWidth = 320;
		int camHeight = 240;
		int imageCenter = (camWidth);
		std::shared_ptr<NetworkTable> myTable = NetworkTable::GetTable(
				"SmartDashboard");

		void RobotInit() override
		{
			roboRealm = NetworkTable::GetTable("cam1");

			cam1 = CameraServer::GetInstance()->StartAutomaticCapture("cam1",
					0);
			cam1.SetResolution(camWidth, camHeight);
			cam1.SetFPS(24);

			LOuter = roboRealm->GetNumber("X1", 1);
//			LInner = roboRealm->GetNumber("LeftInner", 1);
			ROuter = roboRealm->GetNumber("X3", 1);
//			RInner = roboRealm->GetNumber("RightInner", 1);

			shootDistance = roboRealm->GetNumber("Distance", 0); //CHANGE THIS TO THE CORRECT VARIABLE BEING SENT FROM ROBOREALM
		}

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

			// c++ dashboard *dashboard can be changed under settings on the driver station*
			SmartDashboard::PutNumber("RPM Output Top shooter: ",
					shooterTop->GetSpeed());
			SmartDashboard::PutNumber("RPM Output Bottom shooter: ",
					shooterBottom->GetSpeed());
			SmartDashboard::PutNumber("Left drive RPM: ",
					leftDrive1 && leftDrive2->GetSpeed());
			SmartDashboard::PutNumber("Right drive RPM: ",
					rightDrive1 && rightDrive2->GetSpeed());

			if (driveStick->GetRawButton(6))
			{
				if (imageCenter > centerPos)
				{
					SmartDashboard::PutString("Vision Says", "Turn Left!!!");
					leftDrive1->Set(-0.5);
					leftDrive2->Set(-0.5);
					rightDrive1->Set(-0.5);
					rightDrive2->Set(-0.5);
				}
				if (imageCenter < centerPos)
				{
					SmartDashboard::PutString("Vision Says", "Turn Right!!!");
					leftDrive1->Set(0.5);
					leftDrive2->Set(0.5);
					rightDrive1->Set(0.5);
					rightDrive2->Set(0.5);
				}
				if (imageCenter > centerPos - 5 && imageCenter < centerPos + 5)
				{/*
				 if (imageCenter > centerPos2)
				 {
				 SmartDashboard::PutString("Vision Says", "Turn!!!");
				 frontDrive1->Set(-0.25);
				 frontDrive2->Set(-0.25);
				 rearDrive1->Set(-0.25);
				 rearDrive2->Set(-0.25);
				 }
				 if (imageCenter < centerPos2)
				 {
				 SmartDashboard::PutString("Vision Says", "Turn!!!");
				 frontDrive1->Set(0.25);
				 frontDrive2->Set(0.25);
				 rearDrive1->Set(0.25);
				 rearDrive2->Set(0.25);
				 }
				 if (imageCenter > centerPos && imageCenter < centerPos2)
				 {*/
					SmartDashboard::PutString("Vision Says", "Good!!!");
					shooterTop->Set(fabs(sqrt(trajectory)));
					shooterBottom->Set(-(fabs(sqrt(trajectory))));
					//	}
				}
			}
			else
			{
				SmartDashboard::PutString("Vision Says", "NOT PRESSED!!!");
				leftDrive1->Set(leftY - leftX);
				leftDrive2->Set(leftY - leftX);
				rightDrive1->Set(-leftY - leftX);
				rightDrive2->Set(-leftY - leftX);
			}

			// start shooter code
			if (driveStick->GetRawAxis(3))
			{
				shooterTop->Set(-rightTrigger);
				shooterBottom->Set(rightTrigger);
			}
			else
			{
				shooterTop->Set(0);
				shooterBottom->Set(0);
			}

			// fix the encoder code here
			// make one block to control motor
			//if speed too fast set last good speed
			// else speed slow ignore at lower than full throtle

			if (shooterTop->GetSpeed() < trajectoryCal - 2) // trajectoryCal claculates the correct power for the top motor, and -2 for tolerance
			{
				shooterTop++;
			}
			if (shooterTop->GetSpeed() > trajectoryCal + 2) // trajectoryCal claculates the correct power for the top motor, and +2 for tolerance
			{
				shooterTop--;
			}
			if (shooterBottom->GetSpeed() < -trajectoryCal - 2) // -trajectoryCal claculates the correct power for the bottom motor, and -2 for tolerance
			{
				shooterBottom++;
			}
			if (shooterBottom->GetSpeed() > -trajectoryCal + 2) // -trajectoryCal claculates the correct power for the bottom motor, and +2 for tolerance
			{
				shooterBottom--;
			}
			// end of shooter code

			// climb code start
			if (driveStick->GetRawAxis(2) && driveStick->GetRawButton(6))
			{
				climber->Set(leftTrigger);
			}
			else
			{
				climber->Set(0);
			}
			// climb code end

			// start intake code
			if (driveStick->GetRawButton(4)) // pwm 0
			{
				intake->Set(-1);
			}
			else if (driveStick->GetRawButton(1)) // pwm 0
			{
				intake->Set(1);
			}
			else
			{
				intake->Set(0);
			}
			// end intake code

			// start agitator code
			if (driveStick->GetRawButton(2)) // pwm 1
			{
				agitator->Set(-0.5); // test at 1 or 100 percent
			}
			else if (driveStick->GetRawButton(3)) // pwm 1
			{
				agitator->Set(0.5);
			}
			else
			{
				agitator->Set(0);
			}
			// end agitator code
		}

		void TestInit()
		{

		}

		void TestPeriodic()
		{

		}
};

START_ROBOT_CLASS(Robot);
