#include "WPILib.h"
#include "CANTalon.h"

class Robot: public IterativeRobot {

	//Construct for objects here
	Joystick *driveStick;
	Joystick *manipulatorStick;
	CANTalon *frontDrive1;
	CANTalon *frontDrive2;
	CANTalon *rearDrive1;
	CANTalon *rearDrive2;
	CANTalon *leftDrive;
	CANTalon *rightDrive;
	CANTalon *shooter;
	DigitalInput *limitSwitch;
	DigitalInput *shooterButton;
//	DoubleSolenoid *Shifter;

public:
	Robot() {
		Wait(1);
		driveStick = new Joystick(0);
		manipulatorStick = new Joystick(1);
		frontDrive1 = new CANTalon(1);
		frontDrive2 = new CANTalon(2);
		rearDrive1 = new CANTalon(3);
		rearDrive2 = new CANTalon(4);
		leftDrive = new CANTalon(5);
		rightDrive = new CANTalon(6);
		shooter = new CANTalon(7);

		limitSwitch = new DigitalInput(1); //look at the chrome tab this is supposed to get stuff not put out stuff

		shooterButton = new DigitalInput(0);

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
	int slope = 4;
	double accellerationPoint = 47.625;

	//shooter variables
	double maxCount = 65; //this is an arbitrary number for an temporary use
	double goodCount = 60;
	double counter = 0;

	//limit switch variables
	bool pressed = true;

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

		leftX = pow(fabs(leftX) / slope, 2);
		if (driveStick->GetRawAxis(0) > 0) {
			leftX = -leftX;
		}
		/*if(driveStick->GetRawAxis(1) < 0) {
		 leftY = -leftY;
		 }*/

		leftY = pow(fabs(leftY) / slope, 2);
		if (driveStick->GetRawAxis(1) < 0) {
			leftY = -leftY;
		}

		rightX = pow(fabs(rightX) / slope, 2);
		if (driveStick->GetRawAxis(4) > 0) {
			rightX = -rightX;
		}

		rightY = pow(fabs(rightY) / slope, 2);
		if (driveStick->GetRawAxis(5) > 0) {
			rightY = -rightY;
		}

		mRightY = pow(fabs(mRightY) / slope, 2);
		if (manipulatorStick->GetRawAxis(5) > 0) {
			mRightY = -mRightY;
		}

		mLeftY = pow(fabs(mLeftY) / slope, 2);
		if (manipulatorStick->GetRawAxis(1) > 0) {
			mLeftY = -mLeftY;
		}

		rightTrigger = (driveStick->GetRawAxis(2));
		if (fabs(rightTrigger) < (threshold))
			rightTrigger = 0;

		leftTrigger = (driveStick->GetRawAxis(2));
		if (fabs(leftTrigger) < (threshold))
			leftTrigger = 0;

		if (fabs(leftY - leftX) < fabs(accellerationPoint))
			frontDrive2 = 0;

		if (fabs(-leftY - leftX) < fabs(accellerationPoint))
			rearDrive2 = 0;
		// End of code for controller values


		frontDrive1->Set(leftY-leftX);
		frontDrive2->Set(leftY-leftX);
		rearDrive1->Set(-leftY-leftX);
		rearDrive2->Set(-leftY-leftX);


		/*
		frontDrive1->Set(leftY - leftX);
		frontDrive2->Set(leftY - leftX);
		rearDrive1->Set(-leftY - leftX);
		rearDrive2->Set(-leftY - leftX);
		leftDrive->Set(rightX);
		rightDrive->Set(-rightX);
		*/

		if (driveStick->GetRawButton(6)) {
			if (shooterButton->Get()) {
				shooter->Set(127);
			} else {
				shooter->Set(0);
			}

			/*******************************************************************\
		 | pressed: true                                                     |
			 | counter: limit switch pressed                                     |
			 | goodCount: the ideal revolutions the motor turns before resetting |
			 | shooter: ball shooting motor  |
			 |
			 \*******************************/
			/*
			 while (1 == 1) {

			 if (limitSwitch == 1) {
			 if (counter > goodCount) {//...and if the counter has more integers than the goodCount...
			 counter++;		//...add one count to the previous count
			 shooter = shooter - 1;	//...make the shooter motor slower
			 } else if (counter < goodCount) {//...or if the counter has less integers than goodCount...
			 counter++;		//...add one count to the previous count
			 shooter = shooter + 1;		//..make the motor slower
			 } else {		//if the counter is equal to the goodCount...
			 counter++;		//...add one count to the previous count
			 Wait(1);	//change this number (x) to: x/rpm = counter
			 }
			 }
			 }*/

			/*
			 void TestInit() {

			 }

			 void TestPeriodic() {

			 }*/
		}
	}
};
START_ROBOT_CLASS(Robot)
