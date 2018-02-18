#include <iostream>
#include <string>
#include <memory>
#include <DriverStation.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <stdlib.h> 							//standard libraries
#include "WPILib.h"
#include "doublesolenoid.h"
#include "Timer.h"
#include <PIDSource.h>
#include <AnalogGyro.h>
#include <cmath>
#include <ADXRS450_Gyro.h>
#include "Ultrasonic.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <networkTables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <string>
using namespace std;
using namespace nt;
class Robot : public frc::IterativeRobot {
public:
	struct Leg_Data{
		float DIST;
		float ANGLE;
	}Segments[3];
	enum segmentState{
		SEG_1 = 0,
		SEG_2 = 1,
		SEG_3 = 2,
	};
	enum switchpositionValues{
		LEFT_FIELD = 0,
		RIGHT_FIELD = 1,
		FAILURE = -1,
	};
	enum ROBOT_STARTING_POS{
		BOT_ON_LEFT = 0,
		BOT_ON_CENTER = 1,
		BOT_ON_RIGHT = 2,
	};
	bool isHighGear;
	bool soloTest;
	bool targetReached;
	const float encoderRotTick = 4096.0;		//4096 "TICKS" PER ROTATION ACCORDING TO MAGNETIC ENCODER SPECS
	const float PI = 3.1415;
	const float circumference = (PI*6.0);
	const float inchToMeter = (1.0/39.4);
	const float gearRatio = (2.0/15.0);
	const float motorVelocity = 2048.0;
	double robotThrottle;
	double robotSteer;
	double elevatorThrotte;
	double navxGyro;
	double rioGyro;
	double combinedGyroValue;
	int fieldPos;
	TalonSRX *leftMasterMotor;
	TalonSRX *leftSlaveMotor;
	TalonSRX *rightMasterMotor;
	TalonSRX *rightSlaveMotor;
	TalonSRX *elevatorMasterMotor;
	TalonSRX *elevatorSlaveMotor;
	Joystick *leftJoystick;
	Joystick *rightJoystick;

	DoubleSolenoid *gearBox;

	Timer *autonomousTimer;
	AHRS *NAVXBoard;
	ADXRS450_Gyro *ADXGyro;
	AnalogInput *roborioUltrasonic;

	NetworkTableEntry cameraErrorAngle;
	NetworkTableEntry trackingState;
	NetworkTableEntry allianceColor;
	NetworkTableEntry ultrasonicDistance;
	NetworkTableEntry scaleString;
	NetworkTableEntry dataSink;
	void RobotInit() {
		isHighGear = false;
		soloTest = true;
		targetReached = false;
		robotThrottle = 0;
		robotSteer = 0;
		navxGyro = 0;
		rioGyro = 0;
		combinedGyroValue = 0;
		//Temp.
		fieldPos = 0;
		leftMasterMotor = new TalonSRX(1);
		rightMasterMotor = new TalonSRX(3);
		leftSlaveMotor = new TalonSRX(2);
		rightSlaveMotor = new TalonSRX(4);
		rightJoystick = new Joystick(1);
		elevatorMasterMotor = new TalonSRX(5);
		elevatorSlaveMotor = new TalonSRX(6);
        leftMasterMotor->Set(ControlMode::PercentOutput, 0);
        rightMasterMotor->Set(ControlMode::PercentOutput, 0);
        leftSlaveMotor->Set(ControlMode::Follower, 1);
		rightSlaveMotor->Set(ControlMode::Follower, 3);
		gearBox = new DoubleSolenoid(1, 2);
		autonomousTimer = new Timer();
        try {
            NAVXBoard = new AHRS(SPI::Port::kMXP);
        } catch (exception failure ) {
            string errorString = "Error instantiating navX-MXP: NAVXBoard";
            errorString += failure.what();
            DriverStation::ReportError(errorString.c_str());
        }
        ADXGyro = new ADXRS450_Gyro();
        roborioUltrasonic = new AnalogInput(0);
		leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->SetSensorPhase(true);
		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.0075, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.50, 10);
		//Sets up the acceleration and cruise velocity for the MotionMagic Control.
		leftMasterMotor->ConfigMotionCruiseVelocity(2048, 10);
		leftMasterMotor->ConfigMotionAcceleration(1024, 10);

		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSensorPhase(false);
		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.0075, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.50, 10);
		rightMasterMotor->ConfigMotionCruiseVelocity(2048, 10);
		rightMasterMotor->ConfigMotionAcceleration(1024, 10);

		elevatorMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		elevatorMasterMotor->SetSensorPhase(false);
		elevatorMasterMotor->ConfigNominalOutputForward(0, 10);
		elevatorMasterMotor->ConfigNominalOutputReverse(0, 10);
		elevatorMasterMotor->ConfigPeakOutputForward(1, 10);
		elevatorMasterMotor->ConfigPeakOutputReverse(-1, 10);
		elevatorMasterMotor->Config_kF(0, 0.0, 10);
		elevatorMasterMotor->Config_kP(0, 0.0075, 10);
		elevatorMasterMotor->Config_kI(0, 0.0, 10);
		elevatorMasterMotor->Config_kD(0, 0.50, 10);

		autonomousTimer->Reset();
		auto table = NetworkTableInstance::GetDefault();
		auto networkTableData = table.GetTable("Jetson");
		cameraErrorAngle = networkTableData->GetEntry("angle");
		trackingState = networkTableData->GetEntry("V_Working");
		ultrasonicDistance = networkTableData->GetEntry("Distance");
		scaleString = networkTableData->GetEntry("scaleString");
		dataSink = networkTableData->GetEntry("processed");
	}
	void SHIFT_HIGH () {
		gearBox->Set(DoubleSolenoid::kForward);
		frc::Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear=!isHighGear;
	}
	void SHIFT_LOW () {
		gearBox->Set(DoubleSolenoid::kReverse);
		frc::Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear=!isHighGear;
	}
	float ULTRASONIC_CONVERSION() {
		float masterUltrasonicConversion = (5000.0 * 39.4)/4.88;
		float ultrasonicDistance;
		ultrasonicDistance = masterUltrasonicConversion * roborioUltrasonic->GetAverageValue();
		return ultrasonicDistance;
		SmartDashboard::PutNumber("Ultrasonic Distance, Inches", ultrasonicDistance);
	}
	void SEGMENT_SELECTION (double processedData) {
		int switchposition = processedData;
		fieldPos = 2;
		SmartDashboard::PutString("Segment Selection", "Started!");
		switch (switchposition) {
		//switchposition is the variable determining switch position, left or right.
		//Switch on the left side.
		case LEFT_FIELD:
			switch (fieldPos) {
			//fieldPos is the variable determining robot position on the field , left, center, middle.
			case BOT_ON_LEFT:
				//Switch and Case Values have no meaning, yet.
				Segments[0].DIST = 95.0;
				Segments[0].ANGLE = -90.0;
				Segments[1].DIST = 24.0;
				Segments[1].ANGLE = 0.0;
				Segments[2].DIST = 0.0;
				Segments[2].ANGLE = Segments[1].ANGLE;
				SmartDashboard::PutString("Segment Selection", "Robot on Left Field Left!");
				break;
			case BOT_ON_CENTER:
				Segments[0].DIST = 95.0;
				Segments[0].ANGLE = -90.0;
				Segments[1].DIST = 36.0;
				Segments[1].ANGLE = 0.0;
				Segments[2].DIST = 0.0;
				Segments[2].ANGLE = Segments[1].ANGLE;
				SmartDashboard::PutString("Segment Selection", "Robot on Left Field Mid!");

				break;
			case BOT_ON_RIGHT:
				Segments[0].DIST = 95.0;
				Segments[0].ANGLE = -90.0;
				Segments[1].DIST = 48.0;
				Segments[1].ANGLE = 0.0;
				Segments[2].DIST = 0.0;
				Segments[2].ANGLE = Segments[1].ANGLE;
				SmartDashboard::PutString("Segment Selection", "Robot on Left Field Right!");
				break;
			}
			break;
			//Switch on the right side.
		case RIGHT_FIELD:
			switch (fieldPos) {
			case BOT_ON_LEFT:
				Segments[0].DIST = 95.0;
				Segments[0].ANGLE = 90.0;
				Segments[1].DIST = 12.0;
				Segments[1].ANGLE = 0.0;
				Segments[2].DIST = 0.0;
				Segments[2].ANGLE = Segments[1].ANGLE;
				SmartDashboard::PutString("Segment Selection", "Robot on Right Field Left!");

				break;
			case BOT_ON_CENTER:
				Segments[0].DIST = 95.0;
				Segments[0].ANGLE = 90.0;
				Segments[1].DIST = 24.0;
				Segments[1].ANGLE = 0.0;
				Segments[2].DIST = 0.0;
				Segments[2].ANGLE = Segments[1].ANGLE;
				SmartDashboard::PutString("Segment Selection", "Robot on Right Field Mid!");
				break;
			case BOT_ON_RIGHT:
				Segments[0].DIST = 95.0;
				Segments[0].ANGLE = 90.0;
				Segments[1].DIST = 36.0;
				Segments[1].ANGLE = 0.0;
				Segments[2].DIST = 0.0;
				Segments[2].ANGLE = Segments[1].ANGLE;
				SmartDashboard::PutString("Segment Selection", "Robot on Right Field Right!");
				break;
			}
			break;
		case FAILURE:
			SmartDashboard::PutString("Segment Selection", "Total Failure!");
			break;
}

	}
	void DRIVE_TO_DISTANCE(int autonomousDistanceSet) {
		float masterUltrasonicConversion = (4.88 * 39.4)/5000.0;
		rightMasterMotor->SetInverted(true);
		rightSlaveMotor->SetInverted(true);
		rightMasterMotor->SetSensorPhase(true);
		if(((roborioUltrasonic->GetAverageValue() * masterUltrasonicConversion) > 10.0) &&
				(((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick) &&
				(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick)))) {
			leftMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference)) * encoderRotTick);
			rightMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference)) * encoderRotTick);

			SmartDashboard::PutString("Driving State?", "Currently Moving Forward");
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);

			SmartDashboard::PutString("Driving State?", "Done!");
		}
	}
	void TURN_TO_ANGLE (int autonomousAngleSet) {
		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);
		rightMasterMotor->SetSensorPhase(false);

		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);
		leftMasterMotor->Config_kP(0, 0.029, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.00075, 10);
		leftMasterMotor->Config_kF(0, 0.125, 10);

		rightMasterMotor->Config_kP(0, 0.029, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.00075, 10);
		rightMasterMotor->Config_kF(0, 0.125, 10);

		double tolerance = 9.0;
		SmartDashboard::PutBoolean("Reached Angle", targetReached);
		SmartDashboard::PutNumber("Angle Target", autonomousAngleSet);
		SmartDashboard::PutNumber("Combined Value", combinedGyroValue);
		if((!targetReached && (combinedGyroValue < autonomousAngleSet))) {
			if((combinedGyroValue > autonomousAngleSet - tolerance) && (combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			leftMasterMotor->Set(ControlMode::Velocity, (-motorVelocity));
			rightMasterMotor->Set(ControlMode::Velocity, (-motorVelocity));

			SmartDashboard::PutString("Turning State?", "Turning Right!");
		} else if(!targetReached && ((combinedGyroValue > autonomousAngleSet))) {
			if((combinedGyroValue > autonomousAngleSet - tolerance) && (combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			leftMasterMotor->Set(ControlMode::Velocity, (motorVelocity));
			rightMasterMotor->Set(ControlMode::Velocity, (motorVelocity));

			SmartDashboard::PutString("Turning State?", "Turning Left!");
		}
		if(targetReached){
			leftMasterMotor->Set(ControlMode::PercentOutput, (0));
			rightMasterMotor->Set(ControlMode::PercentOutput, (0));

			SmartDashboard::PutString("Turning State?", "Done!");
		}

	}
	void AutonomousInit() override {
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->Set(ControlMode::Position, 0);
		rightMasterMotor->Set(ControlMode::Position, 0);
		SHIFT_LOW();
		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);
		rightMasterMotor->SetSensorPhase(false);
		targetReached = false;

		autonomousTimer->Reset();
		autonomousTimer->Start();
		NAVXBoard->Reset();
		ADXGyro->Reset();
	}

	void AutonomousPeriodic() {

		if(autonomousTimer->Get() < 5.0) {
			DRIVE_TO_DISTANCE(95.0);
			SmartDashboard::PutNumber("Time", autonomousTimer->Get());
		} else if (autonomousTimer->Get() > 5.0) {
			TURN_TO_ANGLE(-90.0);
			SmartDashboard::PutNumber("Time", autonomousTimer->Get());
		}
	}

	void TeleopInit() {
		if(!isHighGear){
			SHIFT_LOW();
		}
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->SetInverted(false);
		leftSlaveMotor->SetInverted(false);
		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);
		rightMasterMotor->SetSensorPhase(false);
		leftMasterMotor->SetSensorPhase(true);
		NAVXBoard->Reset();
		ADXGyro->Reset();
	}

	void TeleopPeriodic() {
		scaleString.SetString(DriverStation::GetInstance().GetGameSpecificMessage());
		SmartDashboard::PutString("Game Data", DriverStation::GetInstance().GetGameSpecificMessage());
		SEGMENT_SELECTION(dataSink.GetDouble(-1));
		if(abs(rightJoystick->GetY()) < 0.05) {
			robotThrottle = 0;
		} else {
			robotThrottle = rightJoystick->GetY();
		}
		if(abs(rightJoystick->GetX()) < 0.05) {
			robotSteer = 0;
		} else {
			robotSteer = rightJoystick->GetX();
		}
		leftMasterMotor->Set(ControlMode::PercentOutput, robotThrottle - robotSteer);
		rightMasterMotor->Set(ControlMode::PercentOutput, -robotThrottle - robotSteer);

		if(rightJoystick->GetRawButton(2) && soloTest) {
			if(isHighGear) {
				SHIFT_HIGH();
			} else if(!isHighGear) {
				SHIFT_LOW();
			}
			soloTest = false;
		}
		if(!rightJoystick->GetRawButton(2)) {
			soloTest = true;
		}
		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);
		SmartDashboard::PutNumber("ADX Gyroscope Value", ADXGyro->GetAngle());
		SmartDashboard::PutNumber("NAVX Board Value", NAVXBoard->GetAngle());
		SmartDashboard::PutNumber("Composite Gyroscope Value", combinedGyroValue);
		SmartDashboard::PutNumber("Vision Working State", trackingState.GetDouble(-1.0));
		SmartDashboard::PutNumber("Camera Error", cameraErrorAngle.GetDouble(180.0));
		SmartDashboard::PutNumber("Switch Postion", dataSink.GetDouble(-1));

		frc::Wait(0.005);
	}

	void TestPeriodic() {

	}
private:
};

START_ROBOT_CLASS(Robot)
