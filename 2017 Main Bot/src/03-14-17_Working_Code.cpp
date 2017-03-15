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
		CANTalon *climb;

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
			climb = new CANTalon(5);
		}

	private:
		LiveWindow *lw = LiveWindow::GetInstance();
		frc::SendableChooser<std::string> chooser;
		const std::string autoNameDefault = "Default";
		const std::string autoNameLeftGear = "Left Gear";
		const std::string autoNameRightGear = "Right Gear";
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
		double margin = 0;
		double shooterPos = 0;
		double shootCount = 0;
		double turnDistance = 0;
		double shootDistance = 0;
		double targetCenter = 200; // change me for the target center *note camera is offset
		double minTargetmargin = 6;
		double maxTargetmargin = -6;
		bool TooClose = false;
		std::shared_ptr<NetworkTable> roboRealm;

		void RobotInit() override
		{
			chooser.AddDefault(autoNameDefault, autoNameDefault);
			chooser.AddObject(autoNameLeftGear, autoNameLeftGear);
			chooser.AddObject(autoNameRightGear, autoNameRightGear);
			SmartDashboard::PutData("Auto Modes", &chooser);

			roboRealm = NetworkTable::GetTable("cam2");

			cam1 = CameraServer::GetInstance()->StartAutomaticCapture("cam2",0);
			cam1.SetResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
			cam1.SetFPS(30);
			cam1.SetBrightness(68);
			cam1.SetExposureManual(4);
			cam1.SetWhiteBalanceManual(3943);
		}

		void AutonomousInit() {
			TooClose = false;
			autoSelected = chooser.GetSelected();

			// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
			std::string autoSelected = SmartDashboard::GetString("Auto Selected", "Default");
			std::cout << "Auto selected: " << autoSelected << std::endl;

			bool leftChecked = SmartDashboard::GetBoolean("DB/Button 0", false);
			bool rightChecked = SmartDashboard::GetBoolean("DB/Button 1", false);
			if (leftChecked){
				autoSelected = autoNameLeftGear;
			}
			if (rightChecked){
				autoSelected = autoNameRightGear;
			}

			if (autoSelected == autoNameLeftGear) {
				// Custom Auto goes here
				Drive(.3);
				Wait(1.5);
				Turn(.3);
				Wait(1.2);
				Drive(0);
			} else if (autoSelected == autoNameRightGear) {
				Drive(.3);
				Wait(1.4);
				Turn(-.3);
				Wait(1.1);
				Drive(0);
			} else {
				// Default Auto goes here
			}
		}

		void AutonomousPeriodic() override // code for auto mode !!!this will run and stop only when disabled!!!
		{
			SmartDashboard::PutString("DB/String 9", autoSelected);
			//targetCenter = SmartDashboard::GetNumber("TargetCenter", 200);

			if (autoSelected == autoNameLeftGear) {
				// Custom Auto goes here
			} else {
				// Default Auto goes here
			}

			if (!TooClose){
				FindGear();
			}
			//Wait(0.5);
		}

		void FindGear() {
			shootDistance = SmartDashboard::GetNumber("Distance", 1);
			LOuter = SmartDashboard::GetNumber("LeftOuter", 1);
			LInner = SmartDashboard::GetNumber("LeftInner", 1);
			RInner = SmartDashboard::GetNumber("RightInner", 1);
			ROuter = SmartDashboard::GetNumber("RightOuter", 1);
			//margin = (ROuter - RInner)*.6;
			//LInner = LOuter + margin;
			//RInner = ROuter - margin;
			/*
			char str0[50];
			sprintf(str0, "%f", LOuter);
			char str1[50];
			sprintf(str1, "%f", LInner);
			char str2[50];
			sprintf(str2, "%f", RInner);
			char str3[50];
			sprintf(str3, "%f", ROuter);
			char str4[50];
			sprintf(str4, "%f", shootDistance);
			SmartDashboard::PutString("DB/String 0", str0);
			SmartDashboard::PutString("DB/String 1", str1);
			SmartDashboard::PutString("DB/String 2", str2);
			SmartDashboard::PutString("DB/String 3", str3);
			SmartDashboard::PutString("DB/String 4", str4);
			*/
//			if (action == 1)
			if (LInner == 0 && RInner == 0){
				SmartDashboard::PutString("Vision Says", "Can't find it");
				SmartDashboard::PutString("DB/String 5", "Can't find it");
				//Drive(0.4);
				//Turn(0.18);
				//Wait(0.1); // this line is the one to check
			} else if (shootDistance > 0 && shootDistance < 2.2){
				SmartDashboard::PutString("Vision Says", "Too Close");
				SmartDashboard::PutString("DB/String 5", "Too Close");
				TooClose = true;
				Drive(0.4);
				Wait(0.8);
				Drive(0.18);
				Wait(0.5);
			}
			else if (targetCenter > ROuter)
			{
				SmartDashboard::PutString("Vision Says", "Turn Left!!!");
				SmartDashboard::PutString("DB/String 5", "Turn Left!!!");
				Turn(-0.2);
				//Wait(0.3);
				//Turn(0);
			}
			else if (targetCenter < LOuter)
			{
				SmartDashboard::PutString("Vision Says", "Turn Right!!!");
				SmartDashboard::PutString("DB/String 5", "Turn Right!!!");
				Turn(0.2);
				//Wait(0.3);
				//Turn(0);
			}
			else if (targetCenter < ROuter && targetCenter > RInner)
			{
				SmartDashboard::PutString("Vision Says", "Turn Left");
				SmartDashboard::PutString("DB/String 5", "Turn Left");
				//TurnCurve(.2,-.02);
				leftDrive1->Set(0.08);
				leftDrive2->Set(0.08);
				rightDrive1->Set(-0.2);
				rightDrive2->Set(-0.2);
				//Wait(0.1);
				//Turn(0);
			}
			else if (targetCenter > LOuter && targetCenter < LInner)
			{
				SmartDashboard::PutString("Vision Says", "Turn Right");
				SmartDashboard::PutString("DB/String 5", "Turn Right");
				//TurnCurve(0.2,.02);
				leftDrive1->Set(0.2);
				leftDrive2->Set(0.2);
				rightDrive1->Set(-0.08);
				rightDrive2->Set(-0.08);
				//Wait(0.1);
				//Turn(0);
			}
			else if (targetCenter < RInner && targetCenter > LInner)
			{
				SmartDashboard::PutString("Vision Says", "Good!!!");
				SmartDashboard::PutString("DB/String 5", "Good!!!");
				Drive(0.2);
				Wait(0.5); //CHANGE THIS!!!!!!!!!!!

				//Drive(0);
			}
		}

		void Turn(double speed) {
			if (speed > 0)
			{
			leftDrive1->Set(speed);
			leftDrive2->Set(speed);
			rightDrive1->Set(0);
			rightDrive2->Set(0);
			}
			else if (speed < 0)
			{
			leftDrive1->Set(0);
			leftDrive2->Set(0);
			rightDrive1->Set(speed);
			rightDrive2->Set(speed);
			}

		}


		void TurnCurve(double speed, double direction) {
			if (direction > 0){
				leftDrive1->Set(speed);
				leftDrive2->Set(speed);
				rightDrive1->Set(0-speed-direction);
				rightDrive2->Set(0-speed-direction);
			} else {
				leftDrive1->Set(speed+direction);
				leftDrive2->Set(speed+direction);
				rightDrive1->Set(0-speed);
				rightDrive2->Set(0-speed);
			}
		}

		void Drive(double speed){
			leftDrive1->Set(speed);
			leftDrive2->Set(speed);
			rightDrive1->Set(0-speed);
			rightDrive2->Set(0-speed);
		}

		void VisionSays(){
			shootDistance = SmartDashboard::GetNumber("Distance", 1);
			LOuter = SmartDashboard::GetNumber("LeftOuter", 1);
			LInner = SmartDashboard::GetNumber("LeftInner", 1);
			ROuter = SmartDashboard::GetNumber("RightOuter", 1);
			RInner = SmartDashboard::GetNumber("RightInner", 1);
//			if (action == 1)

			if (LInner == 0 && RInner == 0){
				SmartDashboard::PutString("Vision Says", "Can't find it");
				SmartDashboard::PutString("DB/String 5", "Can't find it");
			} else if (shootDistance < 2.2){
				SmartDashboard::PutString("Vision Says", "Too Close");
				SmartDashboard::PutString("DB/String 5", "Too Close");
			} else 	if (targetCenter > ROuter)
			{
				SmartDashboard::PutString("Vision Says", "Turn Left!!!");
				SmartDashboard::PutString("DB/String 5", "Turn Left!!!");
			}
			else if (targetCenter < LOuter)
			{
				SmartDashboard::PutString("Vision Says", "Turn Right!!!");
				SmartDashboard::PutString("DB/String 5", "Turn Right!!!");
			}
			else if (targetCenter < ROuter && targetCenter > RInner)
			{
				SmartDashboard::PutString("Vision Says", "Turn Left");
				SmartDashboard::PutString("DB/String 5", "Turn Left");
			}
			else if (targetCenter > LOuter && targetCenter < LInner)
			{
				SmartDashboard::PutString("Vision Says", "Turn Right");
				SmartDashboard::PutString("DB/String 5", "Turn Right");
			}
			else if (targetCenter < RInner && targetCenter > LInner)
			{
				SmartDashboard::PutString("Vision Says", "Good!!!");
				SmartDashboard::PutString("DB/String 5", "Good!!!");
			}
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
				climb->Set(leftTrigger);
			}
			else if (driveStick->GetRawButton(5))
			{
				climb->Set(-0.5);
			}
			else
			{
				climb->Set(0);
			}
			// climb code end

			if (driveStick->GetRawButton(8))
			{
				VisionSays();
			}
		}


};

START_ROBOT_CLASS(Robot);
