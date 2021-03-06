#include "WPILib.h"
#include <ntcore.h>
#include "CANTalon.h"
#include "networkTables/NetworkTable.h"

class Robot: public IterativeRobot
{

		//Construct for objects here
		Joystick *driveStick;
		Joystick *manipulatorStick;
		Talon *frontDrive1;
		Talon *frontDrive2;
		Talon *rearDrive1;
		Talon *rearDrive2;
//	CANTalon *leftDrive;
//	CANTalon *rightDrive;
		Talon *shooter;
		Talon *climber;
		DigitalInput *limitSwitch;
		DigitalInput *shooterButton;
		DigitalInput *climberButton;
//	ADXRS450_Gyro *gyro;
//	ADXL362 *accel;
		DigitalInput *optical;
//	DoubleSolenoid *Shifter;

	public:
		Robot()
		{
			Wait(1);
			driveStick = new Joystick(0);
			manipulatorStick = new Joystick(1);
			frontDrive1 = new Talon(1);
			frontDrive2 = new Talon(2);
			rearDrive1 = new Talon(0);
			rearDrive2 = new Talon(3);
//		leftDrive = new CANTalon(5);
//		rightDrive = new CANTalon(6);
			shooter = new Talon(7);
			climber = new Talon(8);

			limitSwitch = new DigitalInput(1);

			shooterButton = new DigitalInput(0);

			climberButton = new DigitalInput(2);
			//gyro
			//	gyro = new ADXRS450_Gyro();
			//	accel = new ADXL362();

			optical = new DigitalInput(4);
			//Shifter = new DoubleSolenoid (0,1);

			//CameraServer::GetInstance()->SetQuality(50);
			//table = NetworkTable::GetTable("GRIP/myContoursReport");
		}

	private:
		LiveWindow *lw = LiveWindow::GetInstance();
		const std::string autoNameDefault = "Default";
		const std::string autoNameCustom = "My Auto";
		std::string autoSelected;

		//miscellaneous variables
		bool dUp = false;bool dRight = false;bool dDown =
		false;bool dLeft = false;bool dUpR = false;bool dUpL = false;bool dDownR =
		false;bool dDownL = false;bool buttonVal0 = false;bool buttonVal1 =
		false;bool buttonVal2 = false;bool buttonVal3 = false;

		//drive variables
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
		double threshold = .05;
		int slope = 3;

		//shooter variables
		double maxCount = 65; //this is an arbitrary number for the counter (limit switch)
		double goodCount = 60;
		double count = 0;

		/*CHANGE THE FOLLOWING INTEGERS: "45" = theta = angle of the initial velocity from the horizontal plane (degrees)
		 trajectoryCal = the muzzle velocity of the ball
		 shootDistance = distance from shooter to the goal
		 "97.19685036" = the height of the goal in inches
		 "385.82698" = acceleration due to gravity*/

		int trajectoryCal = ((shootDistance * tan(45))
				- ((385.82698) * (shootDistance * shootDistance)
						/ (2 * 97.19685036 * cos(45) * cos(45))));

		//limit switch variables
		bool on = true; //count the button presses
		int limitSwitchState = 0; // the state of the limit switch

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
//		double centerPos2 = (LInner + RInner);
		double shooterPos = 0;
		double shootCount = 0;
		double turnDistance = 0;
		double shootDistance = 0;
		double targetCenter = 150; // change me for the target center *note camera is offset
		double minTargetmargin = 6;
		double maxTargetmargin = -6;
		int camWidth = 320;
		int camHeight = 240;
		int imageCenter = (camWidth);

		//disabled periodic
		int type = 0;
		int typeMod = 0;

		void RobotInit() override
		{
			printf("Hello World!");
//			NetworkTable::SetClientMode();
			NetworkTable::SetIPAddress("10.18.25.2");
//			NetworkTable::Initialize();
			std::shared_ptr<NetworkTable> myTable = NetworkTable::GetTable(
					"SmartDashboard");
			CameraServer::GetInstance()->StartAutomaticCapture();
			roboRealm = NetworkTable::GetTable("cam1");

			cam1 = CameraServer::GetInstance()->StartAutomaticCapture("cam1",
					0);
			cam1.SetResolution(camWidth, camHeight);
			cam1.SetFPS(24);

			LOuter = roboRealm->GetNumber("X1", 1);
//			LInner = roboRealm->GetNumber("LeftInner", 1);
			ROuter = roboRealm->GetNumber("X3", 1);
//			RInner = roboRealm->GetNumber("RightInner", 1);

			/*frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
			 imaqColorThreshold(frame2, frame, 0, 0, Range(0), Range(255), Range(0);*/

//		CameraServer::GetInstance()->SetQuality(30);
//		CameraServer::GetInstance()->StartAutomaticCapture("cam1");
			/*chooser = new SendableChooser();
			 chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
			 chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
			 SmartDashboard::PutData("Auto Modes", chooser);*/

			/*std::cout << "Areas: ";
			 std::vector<double> arr = table->GetNumberArray("area", llvm::ArrayRef<double>());
			 for (unsigned int i = 0; i < arr.size(); i++) {
			 std::cout << arr[i] << " ";
			 }
			 std::cout << std::endl;
			 Wait(1)
			 */

		}

		void AutonomousInit() // code for auto mode !!!this will run and stop only when disabled!!!
		{/*
		 if (action == 2)
		 {
		 COGX = roboRealm->GetNumber("COG_X", -1.0);
		 COGY = roboRealm->GetNumber("COG_Y", -1.0);
		 centerPos = IMAGE_WIDTH / 2 - COGX;
		 turnDistance = centerPos / 10;
		 } // code for auto goes here !!!It will run when auto is enabled on the ds!!!
		 */
		}

		void AutonomousPeriodic()
		{/*
		 //		SmartDashboard::PutNumber("DB/Slider 3", gyro->GetAngle());
		 if (action == 2)
		 {
		 //			turnDistance = (centerPos / 10) - gyro->GetAngle();
		 //			SmartDashboard::PutNumber("DB/Slider 2", shooterPos);
		 if (COGX != -1)
		 {
		 if (turnDistance > 1.5)
		 {
		 SmartDashboard::PutNumber("DB/Slider 2", leftX);
		 frontDrive1->Set(0.5);
		 frontDrive2->Set(0.5);
		 rearDrive1->Set(0.5);
		 rearDrive2->Set(0.5);
		 }
		 else if (turnDistance < -1.5)
		 {
		 frontDrive1->Set(0.5);
		 frontDrive2->Set(0.5);
		 rearDrive1->Set(0.5);
		 rearDrive2->Set(0.5);
		 }
		 else if (turnDistance > -1.5 && turnDistance < 1.5)
		 {
		 shooter->Set(fabs(sqrt(trajectoryCal)));
		 Wait(10);
		 }
		 else
		 {
		 COGX = roboRealm->GetNumber("COG_X", -1.0);
		 COGY = roboRealm->GetNumber("COG_Y", -1.0);
		 centerPos = IMAGE_WIDTH / 2 - COGX;
		 //turnDistance = (centerPos / 10) - gyro->GetAngle();
		 }
		 }
		 else
		 {
		 printf("Aaaaaaand it's broke. ");
		 }
		 // MODIFY THIS CODE FOR THE SHOOTER!!!
		 // AND ADD CODE FOR THE INFRARED SENSOR!!!
		 if(shooterPos < 90) {
		 winch->Set(-1);
		 } else *//*if(fabs(turnDistance) < 1.5) {
		 winch->Set(0);
		 shooter1->Set(-0.75);
		 shooter2->Set(0.75);
		 Wait(0.5);
		 william->Set(DoubleSolenoid::Value::kReverse);
		 } else {
		 winch->Set(-0.05);
		 }
		 if(switch1->Get() == false) {
		 shooterPos = 0;
		 } else {
		 shooterPos = shooterPos - winch->Get();
		 }
		 }*/
		}

		void TeleopInit() // holds code for pre teleop enable
		{/*
		 centerPos = IMAGE_WIDTH / 2 - COGX;
		 turnDistance = centerPos / 10;
		 } // code for auto goes here !!!It will run when auto is enabled on the ds!!!
		 */

		}

		void TeleopPeriodic() // code for the robot to move is placed here (runs when
		// the robot is enabled in teleop mode)
		{
			//CameraServer::GetInstance()->StartAutomaticCapture("cam0");
//		SmartDashboard::PutNumber("DB/Slider 0", accel->GetX());
//		SmartDashboard::PutNumber("DB/Slider 1", accel->GetY());
//		SmartDashboard::PutNumber("DB/Slider 2", accel->GetZ());
//		SmartDashboard::PutNumber("DB/Slider 3", gyro->GetAngle());
			//frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

			//distance = (sonicIn->GetRaw() * inches);
			//sonicOut->Pulse(10);

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

			if (driveStick->GetRawButton(6))
			{
				if (imageCenter > centerPos)
				{
					SmartDashboard::PutString("Vision Says", "Turn Left!!!");
					frontDrive1->Set(-0.5);
					frontDrive2->Set(-0.5);
					rearDrive1->Set(-0.5);
					rearDrive2->Set(-0.5);
				}
				if (imageCenter < centerPos)
				{
					SmartDashboard::PutString("Vision Says", "Turn Right!!!");
					frontDrive1->Set(0.5);
					frontDrive2->Set(0.5);
					rearDrive1->Set(0.5);
					rearDrive2->Set(0.5);
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
					shooter->Set(fabs(sqrt(trajectoryCal)));
					//	}
				}
			}
			else
			{
				SmartDashboard::PutString("Vision Says", "NOT PRESSED!!!");
				frontDrive1->Set(-leftY - leftX);
				frontDrive2->Set(-leftY - leftX);
				rearDrive1->Set(leftY - leftX);
				rearDrive2->Set(leftY - leftX);
			}
			if (driveStick->GetRawButton(7))
			{
				if (climberButton->Get())
				{
					climber->Set(127);
				}
				else
				{
					climber->Set(0);
				}
			}
			if (driveStick->GetRawButton(6))
			{
				if (shooterButton->Get())
				{
					shooter->Set(63.5);
				}
				else
				{
					shooter->Set(0);
				}
			}
			/*******************************************************************\
			 | pressed: true                                                     |
			 | counter: limit switch pressed                                     |
			 | goodCount: the ideal revolutions the motor turns before resetting |
			 | shooter: ball shooting motor                                      |
			 \*******************************************************************/
			/*
			 void TestInit() {
			 }
			 void TestPeriodic() {
			 }
			 */
		}
};

START_ROBOT_CLASS(Robot)
