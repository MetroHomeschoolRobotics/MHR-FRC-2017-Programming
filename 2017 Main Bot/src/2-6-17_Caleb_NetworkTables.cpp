#include "WPILib.h"
#include "CANTalon.h"
#include "CameraServer.h"

class Robot: public IterativeRobot
{

		//Construct for objects here
		Joystick *driveStick;
		Joystick *manipulatorStick;
		CANTalon *frontDrive1;
		CANTalon *frontDrive2;
		CANTalon *rearDrive1;
		CANTalon *rearDrive2;
		CANTalon *climb;
		CANTalon *intake;
		CANTalon *shooter;
		CANTalon *indexer;

	public:
		Robot()
		{
			Wait(1);
			driveStick = new Joystick(0);
			manipulatorStick = new Joystick(1);
			frontDrive1 = new CANTalon(1);
			frontDrive2 = new CANTalon(2);
			rearDrive1 = new CANTalon(3);
			rearDrive2 = new CANTalon(4);
			climb = new CANTalon(5);
			intake = new CANTalon(6);
			shooter = new CANTalon(7);
			indexer = new CANTalon(8);
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
		std::shared_ptr<NetworkTable> roboRealm;

		void RobotInit() override
		{
			CameraServer::GetInstance()->StartAutomaticCapture();
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
			/*if(driveStick->GetRawAxis(1) < 0) {
			 leftY = -leftY;
			 }*/

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
			frontDrive1->Set(leftY - leftX);
			frontDrive2->Set(leftY - leftX);
			rearDrive1->Set(-leftY - leftX);
			rearDrive2->Set(-leftY - leftX);
			// end of drive code

			// climb code start
			if (driveStick->GetRawAxis(5) && driveStick->GetRawButton(6))
			{
				climb->Set(rightY); // rightY is the right joystick on driver xpad moving up and down
			}
			else
			{
				climb->Set(0);
			}

			if (manipulatorStick->GetRawAxis(5)
					&& manipulatorStick->GetRawButton(6))
			{
				climb->Set(mRightY); // mRightY is the right joystick on manip xpad moving up and down
			}
			else
			{
				climb->Set(0);
			}
			// end of climb code

			// drive stick intake
			if (driveStick->GetRawButton(4)) // button Y on xpad
			{
				intake->Set(1);
			}
			else
			{
				intake->Set(0);
			}

			if (driveStick->GetRawButton(1)) // button A on xpad
			{
				intake->Set(-1);
			}
			else
			{
				intake->Set(0);
			}
			// end of drive stick intake

			// manip stick intake
			if (manipulatorStick->GetRawButton(4)) // button Y on xpad
			{
				intake->Set(1);
			}
			else
			{
				intake->Set(0);
			}

			if (manipulatorStick->GetRawButton(1)) // button A on xpad
			{
				intake->Set(1);
			}
			else
			{
				intake->Set(0);
			}
			// end of manip intake

			if (driveStick->GetRawAxis(2))
			{
				shooter->Set(leftTrigger);
				indexer->Set(leftTrigger);

			}
			else
			{
				shooter->Set(0);
				indexer->Set(0);
			}
		}

		void TestInit()
		{

		}

		void TestPeriodic()
		{

		}
};

START_ROBOT_CLASS(Robot);

