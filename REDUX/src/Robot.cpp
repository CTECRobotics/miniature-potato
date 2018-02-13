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
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <string>
using namespace std;
class Robot : public frc::IterativeRobot {
public:
	//Creates three Leg_Data structs as writable segments for use in autonomous control.
	struct Leg_Data{
		int DIST;
		int ANGLE;
	}*Segments[3];
	//For tracking the current segment during autonomous.
	enum segmentState{
		SEG_1 = 0,
		SEG_2 = 1,
		SEG_3 = 2,
	};
	//Designates the left/right field sides as an enum for intuitiveness.
	enum switchpositionValues{
		LEFT_FIELD = 0,
		RIGHT_FIELD = 1,
	};
	//Creates enum values for the joysticks for intuitiveness.
	enum rightJoystickMap{
		INTAKE = 1,
		GEAR_CHANGE = 2,
		ACTUATOR_UP = 5,
		ACTUATOR_DOWN = 3,
		ELEVATOR_UP = 6,
		ELEVATOR_DOWN = 4,
	};
	enum leftJoystickMap{
		OUTAKE = 1,
		WINCH = 6,
	};

	//Creates enum values based on robot positioning for use in autonomous.
	enum ROBOT_STARTING_POS{
		ROB_ON_LEFT = 0,
		ROB_ON_CENTER = 1,
		ROB_ON_RIGHT = 2,
	};
	bool isTracking;
	bool isHighGear;
	bool soloTest;
	int autoMode;
	int fieldPos;
	//Returns an angle from the TX1 calculations.
	//Should be in degrees of error from the geometric center of the target.
	double cameraError;
	//Angle setpoint for turning in autonomous.
	double gyroSetpoint;
	//Gyroscope output of the AHRS NAVX board.
	double navxGyro;
	//Output from the ADX SPI port gyroscope.
	double rioGyro;
	//Averaged value of both gyroscopes.
	double combinedGyroValue;
	double robotThrottle;
	double robotSteer;
	double elevatorThrottle;
	//Variable for use in the TURN_TO_ANGLE function as the setpoint.
	double autonomousAngleSet;
	//Variable for use in the DRIVE_TO_DISTANCE function as the setpoint.
	double autonomousDistanceSet;
	double autonomousRange;
	//Control variable used for scheduling which "segment" to carry out.
	//Whether it be turning, or driving forward.
	int driveState;
	//String given by the DriverStation/FMS, gives color of allied alliance.
	string allianceColor;
	//Quarter second wait time to allow air to flow through the system and shift the gears.
	const float valveWait = 0.25;
	//Proprietary unit for each Encoder rotation, 4096 units per rotation.
	const float encoderRotTick = 4096.0;
	const float PI = 3.1415;
	//Calculation of circumference, used for autonomous driving control.
	const float circumference = (PI*6.0);
	//Calculation of converting returned ultrasonic sensor voltage to meaningful value.
	const float ultrasonicConversion = 5000/4.88;
	//Ratio of gears from Encoder to actual driveshaft.
	const float gearRatio = (2.0/15.0);
	//Talons are arranged in a Master/Slave order when appropriate.
	//Otherwise standalone.
	TalonSRX *leftMasterMotor;
	TalonSRX *leftSlaveMotor;
	TalonSRX *rightMasterMotor;
	TalonSRX *rightSlaveMotor;
	TalonSRX *winchMasterMotor;
	TalonSRX *winchSlaveMotor;
	TalonSRX *intakeMasterMotor;
	TalonSRX *intakeSlaveMotor;
	TalonSRX *elevatorMasterMotor;
	TalonSRX *elevatorSlaveMotor;
	TalonSRX *actuatorMotor;
	TalonSRX *theLonelyMotor;
	DoubleSolenoid *gearBox;
	//Analog ultrasonic sensor plugged into the Roborio.
	AnalogInput *roborioUltrasonic;
	//The SPI port gyroscope, designated by name.
	ADXRS450_Gyro *ADXGyro;
	//NAVX sensor and navigation board.
	AHRS *NAVXBoard;
	Timer *autonomousTimer;
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	DriverStation *driverStation;
	//NetworkTable to pass data to/from the TX1 and Roborio.
	NetworkTable *networkTableData;


	void RobotInit() {
		isTracking = false;
		soloTest = true;
		isHighGear = false;
		autoMode = 0;
		fieldPos = 0;
		gyroSetpoint = 0;
		navxGyro = 0;
		rioGyro = 0;
		combinedGyroValue = 0;
		robotThrottle = 0;
		robotSteer = 0;
		autonomousAngleSet = 0;
		autonomousRange= 0;
		allianceColor = "";

		leftMasterMotor = new TalonSRX(1);
		leftSlaveMotor = new TalonSRX(2);
		rightMasterMotor = new TalonSRX(3);
		rightSlaveMotor = new TalonSRX(4);
		winchMasterMotor = new TalonSRX(5);
		winchSlaveMotor = new TalonSRX(6);
		intakeMasterMotor = new TalonSRX(7);
		intakeSlaveMotor = new TalonSRX(8);
		elevatorMasterMotor = new TalonSRX(9);
		elevatorSlaveMotor = new TalonSRX(10);
		actuatorMotor = new TalonSRX(11);
		theLonelyMotor = new TalonSRX(12);

		gearBox = new DoubleSolenoid(2, 1);
		//This try/catch should connect the NAVX.
		//Will spit out an error message if failed.
        try {
            NAVXBoard = new AHRS(SPI::Port::kMXP);
        } catch (exception failure ) {
            string errorString = "Error instantiating navX-MXP: NAVXBoard";
            errorString += failure.what();
            DriverStation::ReportError(errorString.c_str());
        }
        	//SENSORS, JOYSTICKS, ACCELEROMETERS, ETC.
		leftJoystick = new Joystick(0);
		rightJoystick = new Joystick(1);
		ADXGyro = new ADXRS450_Gyro();
		roborioUltrasonic = new AnalogInput(0);
		autonomousTimer = new Timer();

			//INITIAL SETPOINTS, CALIB, ETC.
		ADXGyro->Reset();
		NAVXBoard->Reset();
		autonomousTimer->Reset();
		ADXGyro->Calibrate();
		//ControlMode::(mode) is now used to determing control method.
		//PercentOutput throttles Talons along -1/1 range.
		//Follower sets a Talons as a "slave" relative to another.
		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		leftSlaveMotor->Set(ControlMode::Follower, 1);
		rightSlaveMotor->Set(ControlMode::Follower, 3);
		winchMasterMotor->Set(ControlMode::PercentOutput, 0);
		winchSlaveMotor->Set(ControlMode::Follower, 5);
		intakeMasterMotor->Set(ControlMode::PercentOutput, 0);
		intakeSlaveMotor->Set(ControlMode::Follower, 7);
		elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
		elevatorSlaveMotor->Set(ControlMode::Follower, 0);
		elevatorSlaveMotor->SetInverted(true);
		actuatorMotor->Set(ControlMode::PercentOutput, 0);
			//INTERNAL PID SETUP

		//The pidxid of the sensor can be found off the configuration of the Roborio.
		//Should be a 0, multiple id values not supported (yet).
		leftMasterMotor->GetSelectedSensorVelocity(0);
		//Selects the sensor type and the channel the sensor is communicating on.
		leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		//Sets up the sensor type, the sensor ID number, DEFINED AS pidIdx, and timeout period.
		//Sets the read direction on the Encoders, with Clockwise being positive.
		leftMasterMotor->SetSensorPhase(true);
		//Sets up min/max output values for the Talons (are these even nessacary?).
		leftMasterMotor->ConfigNominalOutputForward(0, 10);
		leftMasterMotor->ConfigNominalOutputReverse(0, 10);
		leftMasterMotor->ConfigPeakOutputForward(1, 10);
		leftMasterMotor->ConfigPeakOutputReverse(-1, 10);
		//Sets up the various PIDF values to tune the motor output.
		//P = (Desired motor output percentage * 1023)/(error).
		leftMasterMotor->Config_kF(0, 0.029, 10);
		leftMasterMotor->Config_kP(0, 0.0, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.0, 10);

		rightMasterMotor->GetSelectedSensorVelocity(0);
		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		rightMasterMotor->SetSensorPhase(false);
		rightMasterMotor->ConfigNominalOutputForward(0, 10);
		rightMasterMotor->ConfigNominalOutputReverse(0, 10);
		rightMasterMotor->ConfigPeakOutputForward(1, 10);
		rightMasterMotor->ConfigPeakOutputReverse(-1, 10);
		rightMasterMotor->Config_kF(0, 0.029, 10);
		rightMasterMotor->Config_kP(0, 0.0, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.0, 10);

		elevatorMasterMotor->ConfigForwardSoftLimitThreshold(10000, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitThreshold(-10000, 10);
		//Setting up of soft limits, essentially a min/max movement range for the elevator mechanism.
		//The units are defined in CTRE's proprietary units, as in 4096 units per rotation.
		elevatorMasterMotor->ConfigForwardSoftLimitEnable(true, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitEnable(true, 10);

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);


		//Sets up the Roborio into Server Mode.
		networkTableData->SetServerMode();
		//Sets up the Roborio's IP address for the TX1 to communicate on.
		networkTableData->SetIPAddress("roborio-6445-frc.local");
		//Initializes the Networktable.
		networkTableData->Initialize();
		networkTableData->GetEntry("JETSON");
	}
	//Input a turn angle, calculate the position needed, and rotate as such.
	//Output a success statement.
	bool TURN_TO_ANGLE (int autonomousAngleSet) {
		const float inchToMeter = (1.0/39.4);
		const float drivetrainRadius = 12.5625;
		//Distance from the geometric center of the robot to the center contact points of the wheels.
		//Due to West Coast drive.
		//There are 4096 ticks in each rotation, as counted by the encoders.
		const float angletolerance = 2.0;
		float difference = 0.0;
		double direction;
		float distancePerWheel = 0;
		navxGyro = NAVXBoard->GetAngle() + 180;
		rioGyro = ADXGyro->GetAngle() + 180;
		combinedGyroValue = ((navxGyro + rioGyro)/2);
		bool doneTurning;

		difference = ((abs(autonomousAngleSet - combinedGyroValue)) * (PI/180));
		//Angular difference in radians for use in determining direction.
		distancePerWheel = (drivetrainRadius * inchToMeter * difference);
		//Distance needed to be traveled by each side of the robot.
		//Essentially S = r * (theta).
		direction = ((sin(difference)))/abs(sin(difference));

		SmartDashboard::PutNumber("Distance to Drive", distancePerWheel);
		SmartDashboard::PutNumber("Direction", direction);

		if((combinedGyroValue != (autonomousAngleSet - angletolerance)) || (combinedGyroValue != (autonomousAngleSet + angletolerance))) {
			leftMasterMotor->Set(ControlMode::Position, (distancePerWheel/(circumference * inchToMeter)) * encoderRotTick * direction);
			rightMasterMotor->Set(ControlMode::Position, (distancePerWheel/(circumference * inchToMeter)) * encoderRotTick * direction);
			doneTurning = false;
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			doneTurning = true;
		}
return doneTurning;
	}

	bool DRIVE_TO_DISTANCE(int autonomousDistanceSet) {
		const float inchToMeter = (1.0/39.4);
		bool doneDriving;
		if((((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < ((autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick) &&
				(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < ((autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick)))) {
			leftMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);
			rightMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);

			SmartDashboard::PutNumber("Left Encoder Position", (gearRatio)*(leftMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Right Encoder Position", (gearRatio)*(rightMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Distance to Drive", (autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);
			SmartDashboard::PutNumber("Distance in Meters", autonomousDistanceSet);
			SmartDashboard::PutString("Driving State?", "Currently Moving Forward");

			doneDriving = false;
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);

			SmartDashboard::PutNumber("Left Encoder Position", (gearRatio)*(leftMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Right Encoder Position", (gearRatio)*(rightMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutString("Driving State?", "Done!");

			doneDriving = true;
		}
		return doneDriving;
	}

	void SEGMENT_SELECTION (string position) {
		bool LLL = false;
		bool RRR = false;
		bool LRL = false;
		bool RLR = false;
		int switchposition = 0;
		string functionPos;
		functionPos = position;
		if(functionPos.find("LLL") != string::npos) {
			LLL = true;
			//npos checks for whether the find string exists in the to-be parsed.
		} else if(functionPos.find("RRR") == string::npos) {
			RRR = true;
		} else if(functionPos.find("LRL") == string::npos) {
			LRL = true;
		} else if(functionPos.find("RLR") == string::npos) {
			RLR = true;
		}

		if (LLL) {
			switchposition = 0;
			SmartDashboard::PutNumber("Switch", switchposition);
		} else if (RRR) {
			switchposition = 1;
		} else if (LRL) {
			switchposition = 0;
		} else if (RLR) {
			switchposition = 1;
		} else {
			SmartDashboard::PutString("Position State", "Failure to assign state");
			switchposition = -1;
			//TODO Write debug statements.
		}

		switch (switchposition) {
		//switchposition is the variable determining switch position, left or right.
		case LEFT_FIELD:
			switch (fieldPos) {
			//fieldPos is the variable determining robot position on the field , left, center, middle.
			case ROB_ON_LEFT:
				//Switch and Case Values have no meaning, yet.
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case ROB_ON_CENTER:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case ROB_ON_RIGHT:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			}
			break;
		case RIGHT_FIELD:
			switch (fieldPos) {
			case ROB_ON_LEFT:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case ROB_ON_CENTER:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case ROB_ON_RIGHT:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			}
			break;
		}

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
	void AutonomousInit() override {
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->Set(ControlMode::Position, 0);
		rightMasterMotor->Set(ControlMode::Position, 0);
		SHIFT_LOW();
		rightMasterMotor->SetInverted(true);
		rightSlaveMotor->SetInverted(true);
		rightMasterMotor->SetSensorPhase(true);

		autonomousTimer->Reset();
		autonomousTimer->Start();
		NAVXBoard->Reset();
		ADXGyro->Reset();

		allianceColor = to_string(driverStation->GetInstance().GetAlliance());
		SmartDashboard::PutString("Alliance Color", allianceColor);
		networkTableData->PutString("Alliance Color", allianceColor);
		SEGMENT_SELECTION(DriverStation::GetInstance().GetGameSpecificMessage());
		driveState = SEG_1;
	}

	void AutonomousPeriodic() {
		autonomousRange = roborioUltrasonic->GetAverageValue();
		//Selection based on switch position, or robotic position.
		//isTracking will need to be retrieved from networktables.
		//Time and Ultrasonic Distance will need to be sent to Jeff.
		float angle;
		networkTableData->PutNumber("TIME", autonomousTimer->Get());
		networkTableData->PutNumber("DISTANCE", ultrasonicConversion * roborioUltrasonic->GetAverageVoltage());
		angle = networkTableData->GetNumber("ANGLE", -1);
		isTracking = networkTableData->GetBoolean("TRACKING", false);

		bool doneDriving = false;
		bool doneTurning = false;

		if(!isTracking) {
			doneDriving = DRIVE_TO_DISTANCE(Segments[driveState]->DIST);
			if (doneDriving) {
				doneTurning = TURN_TO_ANGLE(Segments[driveState]->ANGLE);
			}
			if (doneDriving && doneTurning) {
				driveState ++;
			}
		} else {
			doneTurning = TURN_TO_ANGLE(angle);
			if(doneTurning) {
				DRIVE_TO_DISTANCE(ultrasonicConversion * roborioUltrasonic->GetAverageValue() - 0.125);
			} else {
//TODO FIND SOMETHING FOR HERE OR
			}
		}
	}

	void TeleopInit() {
		autonomousTimer->Stop();
		NAVXBoard->Reset();
		ADXGyro->Reset();

		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		//This function should aways put us into low gear, regardless of previous gear setting.
		if(!isHighGear){
			SHIFT_LOW();
		}
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->SetInverted(false);
		leftSlaveMotor->SetInverted(false);
		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);
		rightMasterMotor->SetSensorPhase(false);
	}

	void TeleopPeriodic() {
		//Thresholds are set to 5% of the Joystick's range.
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
		if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && soloTest) {
			//soloTest acts as a single limiter to keep the if/else loop from running indefinitely.
			//If the button is pressed, and since soloTest is true, state should run the if/else once
			if(isHighGear) {
				SHIFT_HIGH();
			} else if(!isHighGear) {
				SHIFT_LOW();
			}
		soloTest = false;
		}
		if(!rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
			soloTest = true;
		}
		if(rightJoystick->GetRawButton(rightJoystickMap::ELEVATOR_UP)) {

		}
		if(rightJoystick->GetRawButton(rightJoystickMap::ELEVATOR_DOWN)) {

		}
		if(rightJoystick->GetRawButton(rightJoystickMap::ACTUATOR_UP)) {

		}
		if(rightJoystick->GetRawButton(rightJoystickMap::ACTUATOR_DOWN)) {

		}
		if(rightJoystick->GetRawButton(rightJoystickMap::INTAKE)) {

		}

		if(leftJoystick->GetRawButton(leftJoystickMap::OUTAKE)) {

		}
		if(leftJoystick->GetRawButton(leftJoystickMap::WINCH)) {

		}
		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);

		roborioUltrasonic->GetAverageValue();

		SmartDashboard::PutNumber("ADX_OUTPUT", ADXGyro->GetAngle());
		SmartDashboard::PutNumber("NAVX_OUTPUT", NAVXBoard->GetAngle());
		SmartDashboard::PutNumber("COMPOSITE_OUTPUT", combinedGyroValue);
		SmartDashboard::PutNumber("RANGE", roborioUltrasonic->GetVoltage());

		frc::Wait(0.005);
	}

	void TestPeriodic() {

	}

private:

};

START_ROBOT_CLASS(Robot)
