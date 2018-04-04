#include <iostream>
#include <string>
#include <memory>
#include <DriverStation.h>
#include <IterativeRobot.h>
#include <stdlib.h>
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
using namespace std;
using namespace nt;
class Robot : public frc::IterativeRobot {
public:
	//Creates three Leg_Data structs as writable segments for use in autonomous control.
	struct Leg_Switch_Data{
		double SWITCH_DIST;
		double SWITCH_ANGLE;
		double SWITCH_ACTION;
	}switchSegments[3];

	struct Leg_Scale_Data{
		double SCALE_DIST;
		double SCALE_ANGLE;
		double SCALE_ACTION;
	}scaleSegments[4];
	//Segment enums for tracking the current segment during autonomous.

	enum autonomousSwitchData{
		SWITCH_LEFT = 1,
		SWITCH_RIGHT = 2,
	};

	enum autonomousScaleData{
		SCALE_LEFT = 1,
		SCALE_RIGHT = 2,
	};

	//Designates the left/right field sides as an enum for intuitiveness.

	enum switchpositionValues{
		LEFT_FIELD = 0,
		RIGHT_FIELD = 1,
		FIELD_FAILURE = 2,
	};

	//Creates enum values based on robot positioning for use in autonomous.

	enum robotStartingPosition{
		BOT_ON_LEFT = 0,
		BOT_ON_CENTER = 1,
		BOT_ON_RIGHT = 2,
		BOT_POS_FAILURE = 3,
	};

	//Creates enum values for the joysticks for intuitiveness.
	enum rightJoystickMap{
		ACTUATOR_TOGGLE = 1,
		GEAR_CHANGE = 2,
	};

	enum XboxGamepadMap{
		JOYSTICK_ACTUATOR_UP = 180,
		JOYSTICK_ACTUATOR_DOWN = 0,
		ELEVATOR_AXIS = 1,
		INTAKE_AXIS = 5,
		GAMEPAD_ACTUATOR_TOGGLE = 1,
	};
	//State checkers to ensure proper toggling operation for shifting.
	bool isHighGear;
	bool isInHighGear;
	bool isSoftEnabled;
	//State checkers to ensure proper toggling operation for pneumatic actuating.
	bool isActuator;
	bool isArmIn;
	//State checker for two driver control schemes.
	bool isControl;
	//Overide for the soft limits.
	bool isOverride;
	//Booleans for toggle state testing.
	bool shiftingSoloTest;
	bool actuatorSoloTest;
	bool controlSoloTest;
	bool overideSoloTest;
	//Master toggle for swapping control schemes.
	bool teleopMasterSwitch;
	//Various parameterdriveParamStateSets needed for proper driving function.
	bool isTurnState;
	bool isDriveState;

	bool doneDriving;
	bool doneTurning;
	//This variable is not localized due to need to initialize false and problems with looping.
	bool targetReached;

	//Int to decide whether to be in swtich or scale mode.
	//Also coordinates whether to be in high or low gear.
	int switchScale;

	int autoMode;
	//Int to keep track of robot position on the field when placed.
	int fieldPos;
	//Limiter for autonomous to finish running various segments.
	//At the end of the autonomous code.
	int maxSegmentCount;
	//Control variable used for scheduling which "segment" to carry out.
	//Whether it be turning, or driving forward.
	int driveState;
	//Value for keeping track of which gear ratio to be using.
	double gearRatio;
	//Two throttle values for autonomous.
	double lowGearMotorVelocity;
	double highGearMotorVelocity;
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
	double intakeThrottle;
	double autonomousAngleSet;
	//Variable for use in the DRIVE_TO_DISTANCE function as the setpoint.
	double autonomousDistanceSet;
	double autonomousRange;
	//String given by the DriverStation/FMS, gives color of allied alliance.
	string allianceColor;
	//Quarter second wait time to allow air to flow through the system and shift the gears.
	const float valveWait = 0.25;
	//Previously 2.0/15.0
	//Gear ratios for low/high gears.
	const float standardGearRatio = (1.0/15.0);
//	const float lowGearRatio = (1.0/5.0);
//	const float highGearRatio = (6.0/17.0);
	//Talons are arranged in a Master/Slave order when appropriate.
	//Otherwise standalone.
	TalonSRX *leftMasterMotor;
	TalonSRX *leftSlaveMotor;
	TalonSRX *rightMasterMotor;
	TalonSRX *rightSlaveMotor;
	TalonSRX *elevatorMasterMotor;
	TalonSRX *elevatorSlaveMotor;
	//DoubleSolenoid double actuating valve for shifting mechanism.
	DoubleSolenoid *gearBox;
	//DoubleSolenoid double actuating valve for cube grabbing mechanism.
	DoubleSolenoid *intakeArmActuator;
	//The SPI port gyroscope, designated by name.
	ADXRS450_Gyro *ADXGyro;
	//NAVX sensor and navigation board.///'.;//
	//TODO We do not have a spare NAVX-MXP
	AHRS *NAVXBoard;
	Timer *autonomousTimer;
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	Joystick *XboxGamepad;

//12 to 50 to 34 to 36 to 3
			//24 to 60
//12 to 24 to 60 to 36 to 3
			//24 to 60
	void RobotInit() {
		isHighGear = false;
		isActuator = false;
		isControl = false;
		isOverride = false;

		shiftingSoloTest = true;
		actuatorSoloTest = true;
		controlSoloTest = true;
		overideSoloTest = true;

		isTurnState = true;
		isDriveState = true;

		doneDriving = false;
		doneTurning = false;

		targetReached = false;

		navxGyro = 0;
		rioGyro = 0;
		combinedGyroValue = 0;

		robotThrottle = 0;
		elevatorThrottle = 0;
		intakeThrottle = 0;
		robotSteer = 0;

		leftMasterMotor = new TalonSRX(1);
		leftSlaveMotor = new TalonSRX(2);
		rightMasterMotor = new TalonSRX(3);
		rightSlaveMotor = new TalonSRX(4);

		elevatorMasterMotor = new TalonSRX(5);
		elevatorSlaveMotor = new TalonSRX(7);

		gearBox = new DoubleSolenoid(2, 1);
		intakeArmActuator = new DoubleSolenoid(6, 7);

		//This try/catch should connect the NAVX.
		//Will output an error message if connection failed.
        try {
            NAVXBoard = new AHRS(SPI::Port::kMXP);
        } catch (exception NAVXFailure ) {
            string NAVXErrorString = "Error instantiating NAVXBoard!";
            NAVXErrorString += NAVXFailure.what();
            DriverStation::ReportError(NAVXErrorString.c_str());
        }

        	//SENSORS, JOYSTICKS, ACCELEROMETERS, ETC.
		leftJoystick = new Joystick(0);
		rightJoystick = new Joystick(1);
		XboxGamepad = new Joystick(2);
		ADXGyro = new ADXRS450_Gyro();
		autonomousTimer = new Timer();

			//INITIAL SETPOINTS, CALIB, ETC.
		ADXGyro->Reset();
		NAVXBoard->Reset();
		autonomousTimer->Reset();
		ADXGyro->Calibrate();

		//ControlMode::kmode is now used to determing control method.
		//PercentOutput throttles Talons along -1/1 range.
		//Follower sets a Talons as a "slave" relative to another.
		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);

		//The follower ID must be correct.
		leftSlaveMotor->Set(ControlMode::Follower, 1);
		rightSlaveMotor->Set(ControlMode::Follower, 3);

		elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
		elevatorSlaveMotor->Set(ControlMode::Follower, 5);

		//TalonSRX PID parameter setup.
		//The pidxid of the sensor can be found in the web-configuration page of the Roborio.
		//Should be a 0, multiple id values not supported (yet).
		//Selects the sensor type and the channel the sensor is communicating on.
		leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		//Sets up the sensor type, the sensor ID number, DEFINED AS pidIdx, and timeout period.
		//Sets the read direction on the Encoders, with Clockwise being positive.
		leftMasterMotor->SetSensorPhase(true);
		//Sets up the various PIDF values to tune the motor output.

		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		rightMasterMotor->SetSensorPhase(false);

		elevatorMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		elevatorMasterMotor->SetSensorPhase(false);
		//Initial calculated value was 0.0284.
		elevatorMasterMotor->Config_kP(0, 0.0284, 10);
		//Initial calculated value was 0.000284.
		elevatorMasterMotor->Config_kI(0, 0.00001625, 10);+
		//Initial calculated value was 1.420.
		elevatorMasterMotor->Config_kD(0, 0.225, 10);

		leftMasterMotor->SetInverted(false);
		leftSlaveMotor->SetInverted(false);
		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);

		elevatorMasterMotor->SetInverted(false);
		elevatorMasterMotor->SetInverted(false);

		elevatorSlaveMotor->SetInverted(false);
		elevatorSlaveMotor->SetInverted(false);

		//Elevator drive motors limited to 100% upward, 50% downward.
		elevatorMasterMotor->ConfigPeakOutputForward(1.0, 10);
		elevatorMasterMotor->ConfigPeakOutputReverse(-0.5, 10);

		elevatorSlaveMotor->ConfigPeakOutputForward(1.0, 10);
		elevatorSlaveMotor->ConfigPeakOutputReverse(-0.5, 10);

		//Ensure that the elevator is set to the LOWEST position on the linear slide.
		//Additionally, ensure that there is no slack in the cables.
		elevatorMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
	}
	//Input a turn angle, calculate the position needed, and rotate as such.
	//Output a success statement.
	bool TURN_TO_ANGLE (int autonomousAngleSet) {

		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);

		bool doneTurning;
		double motorVelocity = 0.5;
		double tolerance = 9.0;

		if((!targetReached && (combinedGyroValue < autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
			rightMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
			doneTurning = false;
			if((combinedGyroValue > autonomousAngleSet - tolerance) && (
					combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			SmartDashboard::PutString("Motion State", "Turning Right!");

		} else if(!targetReached && ((combinedGyroValue > autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
			rightMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
			doneTurning = false;
			if((combinedGyroValue > autonomousAngleSet - tolerance) &&
					(combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			SmartDashboard::PutString("Motion State", "Turning Left!");

		}
		if(targetReached){
			leftMasterMotor->Set(ControlMode::PercentOutput, (0));
			rightMasterMotor->Set(ControlMode::PercentOutput, (0));
			doneTurning = true;
			targetReached = true;
			SmartDashboard::PutString("Motion State", "Done Turning!");
		}
		return doneTurning;
	}
	bool DRIVE_TO_DISTANCE(int autonomousDistanceSet) {
		//Proprietary unit for each Encoder rotation, 4096 units per rotation.
		const float encoderRotTick = 4096.0;
		const float PI = 3.1415;
		//Calculation of circumference, used for autonomous driving control.
		const float circumference = (PI*6.0);
		//Calculation of converting returned ultrasonic sensor voltage to meaningful value.
//		const float masterUltrasonicConversion = (5000.0 * 39.4)/4.88;

		bool doneDriving;

		if((((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick) &&
				(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick)))) {

			leftMasterMotor->Set(ControlMode::PercentOutput, lowGearMotorVelocity);
			rightMasterMotor->Set(ControlMode::PercentOutput, lowGearMotorVelocity);

			SmartDashboard::PutString("Motion State", "Currently Moving Forward!");
			//When doneDriving is false, should continue driving along.
			doneDriving = false;
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);

			SmartDashboard::PutString("Motion State", "Done Forward!");
			//When doneDrving is true, should schedule the next segment.
			doneDriving = true;
		}
		return doneDriving;
	}
	void DATA_SET(string inputData) {
		int switchPos;
		int scalePos;

		//1 = switch autonomous, 2 = scale autonomous.
		switchScale = SmartDashboard::GetNumber("Switch Or Scale", -1);

		if(inputData == "LLL") {
			switchPos = 1;
			scalePos = 1;
		} else if(inputData == "RRR") {
			switchPos = 0;
			scalePos = 0;
		} else if(inputData == "LRL") {
			switchPos = 1;
			scalePos = 0;
		} else if(inputData == "RLR") {
			switchPos = 0;
			scalePos = 1;
		} else {
			switchPos = -1;
			scalePos = -1;
		}

		switch(switchScale) {
		case 1:
			switch(switchPos) {
			case 1:
				switchSegments[0].SWITCH_DIST = 1.0;
				switchSegments[0].SWITCH_ANGLE = 90.0;
				switchSegments[1].SWITCH_DIST = 1.0;
				switchSegments[1].SWITCH_ANGLE = 90.0;
				switchSegments[2].SWITCH_DIST = 1.0;
				switchSegments[2].SWITCH_ANGLE = 90.0;
				break;
			case 2:
				switchSegments[0].SWITCH_DIST = 1.0;
				switchSegments[0].SWITCH_ANGLE = 90.0;
				switchSegments[1].SWITCH_DIST = 1.0;
				switchSegments[1].SWITCH_ANGLE = 90.0;
				switchSegments[2].SWITCH_DIST = 1.0;
				switchSegments[2].SWITCH_ANGLE = 90.0;
				break;
			case -1:
				switchSegments[0].SWITCH_DIST = 0.0;
				switchSegments[0].SWITCH_ANGLE = 0.0;
				switchSegments[1].SWITCH_DIST = 0.0;
				switchSegments[1].SWITCH_ANGLE = 0.0;
				switchSegments[2].SWITCH_DIST = 0.0;
				switchSegments[2].SWITCH_ANGLE = 0.0;
				break;
			}
			break;
		case 2:
			switch(scalePos) {
			case 1:
				switchSegments[0].SWITCH_DIST = 1.0;
				switchSegments[0].SWITCH_ANGLE = 90.0;
				switchSegments[1].SWITCH_DIST = 1.0;
				switchSegments[1].SWITCH_ANGLE = 90.0;
				switchSegments[2].SWITCH_DIST = 1.0;
				switchSegments[2].SWITCH_ANGLE = 90.0;
				switchSegments[3].SWITCH_DIST = 1.0;
				switchSegments[3].SWITCH_ANGLE = 90.0;
				break;
			case 2:
				switchSegments[0].SWITCH_DIST = 1.0;
				switchSegments[0].SWITCH_ANGLE = 90.0;
				switchSegments[1].SWITCH_DIST = 1.0;
				switchSegments[1].SWITCH_ANGLE = 90.0;
				switchSegments[2].SWITCH_DIST = 1.0;
				switchSegments[2].SWITCH_ANGLE = 90.0;
				switchSegments[3].SWITCH_DIST = 1.0;
				switchSegments[3].SWITCH_ANGLE = 90.0;
				break;
			case -1:
				switchSegments[0].SWITCH_DIST = 0.0;
				switchSegments[0].SWITCH_ANGLE = 0.0;
				switchSegments[1].SWITCH_DIST = 0.0;
				switchSegments[1].SWITCH_ANGLE = 0.0;
				switchSegments[2].SWITCH_DIST = 0.0;
				switchSegments[2].SWITCH_ANGLE = 0.0;
				switchSegments[3].SWITCH_DIST = 1.0;
				switchSegments[3].SWITCH_ANGLE = 90.0;
				break;
			}
			break;
		case -1:
			switchSegments[0].SWITCH_DIST = 0.0;
			switchSegments[0].SWITCH_ANGLE = 0.0;
			switchSegments[1].SWITCH_DIST = 0.0;
			switchSegments[1].SWITCH_ANGLE = 0.0;
			switchSegments[2].SWITCH_DIST = 0.0;
			switchSegments[2].SWITCH_ANGLE = 0.0;
			break;
		}
	}

	void SHIFT_HIGH () {
		gearBox->Set(DoubleSolenoid::kForward);
		Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
		//Where green is, and red is.
		isInHighGear = false;
		SmartDashboard::PutBoolean("Gear State", isInHighGear);
	}
	void SHIFT_LOW () {
		gearBox->Set(DoubleSolenoid::kReverse);
		Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
		isInHighGear = true;
		SmartDashboard::PutBoolean("Gear State", isInHighGear);
	}

	void AutonomousInit() override {
		SHIFT_HIGH();
		DATA_SET(DriverStation::GetInstance().GetGameSpecificMessage());

		//O = switch, 1 = scale.
		switchScale = SmartDashboard::GetNumber("Scale?", 0);
		if(switchScale == 0) {
			SHIFT_LOW();
			gearRatio = lowGearRatio;
		} else if(switchScale == 1) {
			SHIFT_HIGH();
			gearRatio = highGearRatio;
		}

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);

		autonomousTimer->Reset();
		autonomousTimer->Start();
		NAVXBoard->Reset();
		ADXGyro->Reset();

		driveState = 0;
		doneDriving = false;
		doneTurning = false;
	}

	void AutonomousPeriodic() {

		if(autonomousTimer->Get() < 2.0) {
//			actuatorMotor->Set(ControlMode::PercentOutput, 0.125);
		} else if(autonomousTimer->Get() < 15.0) {
//			elevatorMasterMotor->Set(ControlMode::Position, 10000);
			if(!doneDriving && !doneTurning) {
				doneDriving = DRIVE_TO_DISTANCE(switchSegments[driveState].SWITCH_DIST);
			} else if(doneDriving && !doneTurning) {
				doneTurning = TURN_TO_ANGLE(switchSegments[driveState].SWITCH_ANGLE);
			} else if(doneDriving && doneTurning) {
				driveState ++;
				leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
				rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
				doneDriving = false;
				doneTurning = false;
			}
		}
	}
	void TeleopInit() {
		autonomousTimer->Stop();
//		NAVXBoard->Reset();
		ADXGyro->Reset();

		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		//This function should always put us into low gear, regardless of previous gear setting.
		SHIFT_LOW();
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);

		leftMasterMotor->SetSensorPhase(true);
		rightMasterMotor->SetSensorPhase(false);

	}
	void TeleopPeriodic() {

	//Checks if the control scheme is set to true or false.
	//True =, false =.
			if(leftJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && shiftingSoloTest) {
				if(isHighGear) {
					SHIFT_HIGH();
				} else if(!isHighGear) {
					SHIFT_LOW();
				}
				shiftingSoloTest = false;
			}
			if(!leftJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
				shiftingSoloTest = true;
			}

			//Thresholds are set to 5% of the Joystick's range.
			if(abs(leftJoystick->GetY()) < 0.05) {
				robotThrottle = 0;
			} else {
				robotThrottle = leftJoystick->GetY();
			}
			if(abs(leftJoystick->GetX()) < 0.05) {
				robotSteer = 0;
			} else {
				robotSteer = leftJoystick->GetX();
			}
			leftMasterMotor->Set(ControlMode::PercentOutput, robotThrottle - robotSteer);
			rightMasterMotor->Set(ControlMode::PercentOutput, -robotThrottle - robotSteer);

			SmartDashboard::PutNumber("Left Encoder Position", standardGearRatio * leftMasterMotor->GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("Right Encoder Position", standardGearRatio * rightMasterMotor->GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("Gyro Value", ADXGyro->GetAngle());
	}
	void TestPeriodic() {

	}
private:
};
START_ROBOT_CLASS(Robot)
