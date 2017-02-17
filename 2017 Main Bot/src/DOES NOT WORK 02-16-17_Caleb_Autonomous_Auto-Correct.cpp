#include "WPILib.h"
#include <ntcore.h>
#include "CANTalon.h"
#include "networkTables/NetworkTable.h"
#include "CameraServer.h"
#include "math.h"

class Robot: public IterativeRobot
{

		//Construct for objects here
		Joystick *driveStick;
		Joystick *manipulatorStick;
		CANTalon *rightDrive1;
		CANTalon *rightDrive2;
		CANTalon *leftDrive1;
		CANTalon *leftDrive2;
//	CANTalon *leftDrive;
//	CANTalon *rightDrive;
		CANTalon *climber;
		CANTalon *shooter;
		CANTalon *indexer;
		CANTalon *intake;
//		CANTalon *shooter2;
		DigitalInput *limitSwitch;
		DigitalInput *shooterButton;
		DigitalInput *climberButton;
		DigitalInput *optical;
//	DoubleSolenoid *Shifter;

	public:
		Robot()
		{
			Wait(1);
			driveStick = new Joystick(0);
			manipulatorStick = new Joystick(1);
			rightDrive1 = new CANTalon(1);
			rightDrive2 = new CANTalon(2);
			leftDrive1 = new CANTalon(4);
			leftDrive2 = new CANTalon(3);
//		leftDrive = new CANTalon(5);
//		rightDrive = new CANTalon(6);
			climber = new CANTalon(5);
			shooter = new CANTalon(6);
			indexer = new CANTalon(7);
			intake = new CANTalon(8);
//			shooter2 = new CANTalon(7);

			limitSwitch = new DigitalInput(1);

			shooterButton = new DigitalInput(0);

			climberButton = new DigitalInput(2);

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
		double threshold = 0.05;
		int slope = 3;

		//shooter variables
		double maxCount = 65; //this is an arbitrary number for an temporary use
		double goodCount = 60;
		double count = 0;

		int trajectoryCal = ((shootDistance * tan(45))
				- ((385.82698) * (shootDistance * shootDistance)
						/ (2 * 2.46888 * cos(45) * cos(45))));

		//limit switch variables
		bool on = true; //count the button presses
		int limitSwitchState = 0; // the state of the limit switch

		//vision tracking
		int action = 1;
		double IMAGE_WIDTH = 320;
		double IMAGE_HEIGHT = 240;
		double COGX = 0;
		double COGY = 0;
		double centerPosX = 0;
		double centerPosY = 0;
		double turnDistance = 0;
		double shootDistance = 0;
		double shooterPos = 0;
		double shootCount = 0;
		double targetCenter = 150; // change me for the target center *note camera is offset
		double minTargetmargin = 6;
		double maxTargetmargin = -6;
		int camWidth = 320;
		int camHeight = 240;
		int imageCenter = (camWidth / 2);
		std::shared_ptr<NetworkTable> roboRealm;
		cs::UsbCamera cam1;
		cs::CvSink highGoalVid;

		//disabled periodic
		int type = 0;
		int typeMod = 0;

		void RobotInit() override
		{
			printf("Hello World!");
			//NetworkTable::SetClientMode();
			//NetworkTable::SetIPAddress("10.18.25.92");
			//NetworkTable::Initialize();
			//cam1 = new cs::UsbCamera("cam1", 1);
			//cam1.SetResolution(camWidth, camHeight);
			//cam1.SetFPS(16);

			cam1 = CameraServer::GetInstance()->StartAutomaticCapture("cam1",
					0);
			cam1.SetResolution(camWidth, camHeight);
			cam1.SetFPS(24);
			//highGoalVid = CameraServer::GetInstance()->GetVideo();
			//highGoalVid.SetEnabled(true);
			std::shared_ptr<NetworkTable> myTable = NetworkTable::GetTable(
					"SmartDashboard");
			//CameraServer::GetInstance()->StartAutomaticCapture();
			//roboRealm = NetworkTable::GetTable("cam1");

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
		{

			if (action == 2)
			{
				COGX = roboRealm->GetNumber("X1", -1.0);
				COGY = roboRealm->GetNumber("Y1", -1.0);
				centerPosX = IMAGE_WIDTH / 2 - COGX;
				centerPosY = IMAGE_HEIGHT / 2 - COGY;
				turnDistance = centerPosX / 10;
			} // code for auto goes here !!!It will run when auto is enabled on the ds!!!
		}

		void AutonomousPeriodic()
		{
			SmartDashboard::PutNumber("DB/Slider 2", shooterPos);
			COGX = roboRealm->GetNumber("X1", -1.0);
			COGY = roboRealm->GetNumber("Y1", -1.0);
			centerPosX = IMAGE_WIDTH / 2 - COGX;
			centerPosY = IMAGE_HEIGHT / 2 - COGY;
			turnDistance = (centerPosX / 10);
			shootDistance = (centerPosY / 10);


			if (action == 2)
			{
				if (COGX != -1)
				{
					if (turnDistance > 1)
					{
						rightDrive1->Set(0.5);
						rightDrive2->Set(0.5);
						leftDrive1->Set(0.5);
						leftDrive2->Set(0.5);
					}
					else if (turnDistance < -1)
					{
						rightDrive1->Set(-0.5);
						rightDrive2->Set(-0.5);
						leftDrive1->Set(-0.5);
						leftDrive2->Set(-0.5);
					}
					else if (turnDistance > -1.5 && turnDistance < 1.5)
					{
						shooter->Set(fabs(sqrt(trajectoryCal)));
					}
				}
			}
			else
			{
				printf("Aaaaaaand it's broke. ");
			}

			// MODIFY THIS CODE FOR THE SHOOTER!!!
			// AND ADD CODE FOR THE INFRARED SENSOR!!!

			/*if(shooterPos < 90) {
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
			 }*/
		}

		void TeleopInit() // holds code for pre teleop enable
		{
			COGX = roboRealm->GetNumber("X1", -1.0);
			COGY = roboRealm->GetNumber("Y1", -1.0);
			centerPosX = IMAGE_WIDTH / 2 - COGX;
			centerPosY = IMAGE_HEIGHT / 2 - COGY;
			turnDistance = (centerPosX / 10);
			shootDistance = (centerPosY / 10);
		}

		void TeleopPeriodic() // code for the robot to move is placed here (runs when
// the robot is enabled in teleop mode)
		{

			COGX = roboRealm->GetNumber("X1", -1.0);
			COGY = roboRealm->GetNumber("Y1", -1.0);
			centerPosX = IMAGE_WIDTH / 2 - COGX;
			centerPosY = IMAGE_HEIGHT / 2 - COGY;
			turnDistance = (centerPosX / 10);
			shootDistance = (centerPosY / 10);
			SmartDashboard::PutNumber("DB/Slider 2", shooterPos);

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

			rightDrive1->Set(leftY - leftX);
			rightDrive2->Set(leftY - leftX);
			leftDrive1->Set(-leftY - leftX);
			leftDrive2->Set(-leftY - leftX);

			if (driveStick->GetRawButton(6))
			{
				if (shooterButton->Get())
				{
					if (COGX != -1)
					{
						if (turnDistance > 1)
						{
							rightDrive1->Set(0.5);
							rightDrive2->Set(0.5);
							leftDrive1->Set(0.5);
							leftDrive2->Set(0.5);
						}
						else if (turnDistance < -1)
						{
							rightDrive1->Set(-0.5);
							rightDrive2->Set(-0.5);
							leftDrive1->Set(-0.5);
							leftDrive2->Set(-0.5);
						}
						else if (turnDistance > -1.5 && turnDistance < 1.5)
						{
							shooter->Set(fabs(sqrt(trajectoryCal)));
						}
					}
				}
			}
			else
			{
				shooter->Set(0);
			}

			if (driveStick->GetRawButton(6))
			{
				climber->Set(rightY); // rightY is the right joystick on driver xpad moving up and down
			}
			else
			{
				climber->Set(0);
			}

			if (manipulatorStick->GetRawButton(6))
			{
				climber->Set(mRightY); // mRightY is the right joystick on manip xpad moving up and down
			}
			else
			{
				climber->Set(0);
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
			}
			else
			{
				shooter->Set(0);
			}

			if (driveStick->GetRawAxis(3))
			{
				indexer->Set(rightTrigger);
			}
			else
			{
				indexer->Set(0);
			}

			if (manipulatorStick->GetRawAxis(2))
			{
				shooter->Set(mLeftTrigger);
			}
			else
			{
				shooter->Set(0);
			}

			if (manipulatorStick->GetRawAxis(3))
			{
				indexer->Set(mRightTrigger);
			}
			else
			{
				indexer->Set(0);
			}

			/*
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
			 */

			/*******************************************************************\
			 | pressed: true                                                     |
			 | counter: limit switch pressed                                     |
			 | goodCount: the ideal revolutions the motor turns before resetting |
			 | shooter: ball shooting motor                                      |
			 \*******************************************************************/
			while (on)
			{
				if (limitSwitchState == true)
				{	//if the limit switch is pressed...
					count++;	//...and count the presses for 1 second
					if (count > 50)
					{	//...but if the "count" is over 48...
						shooter = shooter - 3;//...subtract 3 from the integer that the shooter is "powered" from...
						count = 0;	//...and set the counter to 0
					}
					else if (count < 46)
					{		//...but if the "count" is less than 48...
						shooter = shooter + 2;//...add 2 from the integer that the shooter is powered from...
						count = 0;		//...and set the "count" to 0
					}
					Wait(1); //do this for 1 second...
				}
				count = 0; //...and then reset the count to 0
			}
			/*
			 void TestInit() {
			 }
			 void TestPeriodic() {
			 }
			 */
		}
};

START_ROBOT_CLASS(Robot)
