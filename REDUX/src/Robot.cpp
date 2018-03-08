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
	//Creates three Leg_Data structs as writable segments for use in autonomous control.
	struct Leg_Data{
		double DIST;
		double ANGLE;
		double ACTION;
	}Segments[6];
	//Segment enums for tracking the current segment during autonomous.
	enum segmentState{
		SEG_1 = 0,
		SEG_2 = 1,
		SEG_3 = 2,
		SEG_4 = 3,
		SEG_5 = 4,
		SEG_6 = 5,
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
	enum leftJoystickMap{

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
	//State checkers to ensure proper toggling operation for pneumatic actuating.
	bool isActuator;
	bool isArmIn;
	//State checker for two driver control schemes.
	bool isControl;
	//Booleans for toggle state testing.
	bool shiftingSoloTest;
	bool actuatorSoloTest;
	bool controlSoloTest;
	//Master toggle for swapping control schemes.
	bool teleopMasterSwitch;
	//Various parameters needed for proper driving function.
	bool isTurnState;
	bool isDriveState;

	bool doneDriving;
	bool doneTurning;

	bool targetReached;

	int autoMode;
	//Int to keep track of robot position on the field when placed.
	int fieldPos;
	//Control variable used for scheduling which "segment" to carry out.
	//Whether it be turning, or driving forward.
	int driveState;
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
	double elevatorJoystickThrottle;
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
	//Talons are arranged in a Master/Slave order when appropriate.
	//Otherwise standalone.
	TalonSRX *leftMasterMotor;
	TalonSRX *leftSlaveMotor;
	TalonSRX *rightMasterMotor;
	TalonSRX *rightSlaveMotor;
	TalonSRX *elevatorMasterMotor;
	TalonSRX *elevatorSlaveMotor;
	TalonSRX *intakeMasterMotor;
	TalonSRX *intakeSlaveMotor;
	TalonSRX *actuatorMotor;
	//DoubleSolenoid double actuating valve for shifting mechanism.
	DoubleSolenoid *gearBox;
	//DoubleSolenoid double actuating valve for cube grabbing mechanism.
	DoubleSolenoid *intakeArmActuator;
	//The SPI port gyroscope, designated by name.
	ADXRS450_Gyro *ADXGyro;
	//NAVX sensor and navigation board.
	AHRS *NAVXBoard;
	Timer *autonomousTimer;
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	Joystick *XboxGamepad;
	//Creation of NetworkTableEntries for entries in the NetworkTable.
	//Note the lack of a *, this is because of interaction between pointer and NetworkTableEntry.
//	NetworkTableEntry cameraErrorAngle;
//	NetworkTableEntry trackingState;
//	NetworkTableEntry ultrasonicDistance;
//	NetworkTableEntry dataSink;
//	NetworkTableEntry scaleString;
	void RobotInit() {
		isHighGear = false;
		isActuator = false;
		isControl = false;

		shiftingSoloTest = true;
		actuatorSoloTest = true;
		controlSoloTest = true;

		isTurnState = true;
		isDriveState = true;

		doneDriving = false;
		doneTurning = false;

		targetReached = false;

		autoMode = 0;
		fieldPos = 0;

		gyroSetpoint = 0;
		navxGyro = 0;
		rioGyro = 0;
		combinedGyroValue = 0;

		robotThrottle = 0;
		elevatorThrottle = 0;
		intakeThrottle = 0;
		robotSteer = 0;

		autonomousAngleSet = 0;

		leftMasterMotor = new TalonSRX(1);
		leftSlaveMotor = new TalonSRX(2);
		rightMasterMotor = new TalonSRX(3);
		rightSlaveMotor = new TalonSRX(4);

		elevatorMasterMotor = new TalonSRX(5);
		elevatorSlaveMotor = new TalonSRX(7);

		intakeMasterMotor = new TalonSRX(6);
		intakeSlaveMotor = new TalonSRX(8);

		actuatorMotor = new TalonSRX(9);

		gearBox = new DoubleSolenoid(2, 1);
		intakeArmActuator = new DoubleSolenoid(6, 7);
		//This try/catch should connect the NAVX.
		//Will spit out an error message if failed.
        try {
            NAVXBoard = new AHRS(SPI::Port::kMXP);
        } catch (exception failure ) {
            string errorString = "Error instantiating NAVXBoard";
            errorString += failure.what();
            DriverStation::ReportError(errorString.c_str());
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
		leftSlaveMotor->Set(ControlMode::Follower, 1);
		rightSlaveMotor->Set(ControlMode::Follower, 3);
		elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
		elevatorSlaveMotor->Set(ControlMode::Follower, 5);
		intakeMasterMotor->Set(ControlMode::PercentOutput, 0);
		intakeSlaveMotor->Set(ControlMode::Follower, 6);
		actuatorMotor->Set(ControlMode::PercentOutput, 0);
		//TalonSRX PID parameter setup.
		//The pidxid of the sensor can be found in the web-configuration page of the Roborio.
		//Should be a 0, multiple id values not supported (yet).
		//Selects the sensor type and the channel the sensor is communicating on.
		leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		//Sets up the sensor type, the sensor ID number, DEFINED AS pidIdx, and timeout period.
		//Sets the read direction on the Encoders, with Clockwise being positive.
		leftMasterMotor->SetSensorPhase(true);
		//Sets up the various PIDF values to tune the motor output.
		//P = (Desired motor output percentage * 1023)/(error).
		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.0085, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.50, 10);

		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		rightMasterMotor->SetSensorPhase(false);
		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.0075, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.50, 10);

		elevatorMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		//Limits the output to 75% to ensure no elevator breakage.
		elevatorMasterMotor->ConfigPeakOutputForward(0.5, 10);
		elevatorMasterMotor->ConfigPeakOutputReverse(-0.5, 10);
		elevatorSlaveMotor->ConfigPeakOutputForward(0.5, 10);
		elevatorSlaveMotor->ConfigPeakOutputReverse(-0.5, 10);
		elevatorMasterMotor->SetSensorPhase(false);
		elevatorMasterMotor->Config_kF(0, 0.0, 10);
		elevatorMasterMotor->Config_kP(0, 0.00075, 10);
		elevatorMasterMotor->Config_kI(0, 0.0, 10);
		elevatorMasterMotor->Config_kD(0, 0.00050, 10);

		intakeMasterMotor->ConfigPeakOutputForward(0.5, 10);
		intakeMasterMotor->ConfigPeakOutputReverse(-0.5, 10);

		//TODO If the motors do spin opposing to desired direction, invert!
		elevatorMasterMotor->SetInverted(false);
		elevatorMasterMotor->SetInverted(false);
		elevatorSlaveMotor->SetInverted(false);
		elevatorSlaveMotor->SetInverted(false);

		intakeSlaveMotor->SetInverted(true);
		actuatorMotor->SetInverted(true);

		elevatorMasterMotor->ConfigForwardSoftLimitThreshold(10000, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitThreshold(-10000, 10);
		//Setting up of soft limits, essentially a min/max movement range for the elevator mechanism.
		//The units are defined in CTRE's proprietary units, as in 4096 units per rotation.
		elevatorMasterMotor->ConfigForwardSoftLimitEnable(false, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitEnable(false, 10);

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		//Ensure that the elevator is set to the LOWEST position on the linear slide.
		//Additionally, ensure that there is no slack in the cables.
		elevatorMasterMotor->SetSelectedSensorPosition(0, 0, 10);
	}
	//Input a turn angle, calculate the position needed, and rotate as such.
	//Output a success statement.
	bool TURN_TO_ANGLE (int autonomousAngleSet) {

		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);
		rightMasterMotor->SetSensorPhase(false);

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
		//Ratio of gears from Encoder to actual driveshaft.
		const float gearRatio = (2.0/15.0);
		bool doneDriving;
		double motorVelocity = 0.5;

		rightMasterMotor->SetInverted(true);
		rightSlaveMotor->SetInverted(true);
		rightMasterMotor->SetSensorPhase(true);
		if((((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick) &&
				(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick)))) {
			//Does the calculations necessary to set a position for the motors.
			//autonomousDistanceSet/circumference = number of turns necessary.
			//Number of turns * encRotTick (4096) = position needed to drive to in native units.
//			leftMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference)) * encoderRotTick);
//			rightMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference)) * encoderRotTick);

			leftMasterMotor->Set(ControlMode::PercentOutput, motorVelocity);
			rightMasterMotor->Set(ControlMode::PercentOutput, -motorVelocity);

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
	void SEGMENT_DATA_SET (string inputData) {

		//switchPosition indicates where the switch is located, 0 = left, 1 = right.
		bool LLL = false;
		bool RRR = false;
		bool LRL = false;
		bool RLR = false;
        bool left = SmartDashboard::GetBoolean("Left", !!0);
        bool center = SmartDashboard::GetBoolean("Center", !!0);
        bool right = SmartDashboard::GetBoolean("Right", !!0);
        bool runScale = SmartDashboard::GetBoolean("BaseLine-Scale", !!0);
        bool runSwitch = SmartDashboard::GetBoolean("BaseLine-Switch", !!0);
//        bool runAll = SmartDashboard::GetBoolean("BaseLne-Scale&Switch", !!0);
//        bool runBaseline = SmartDashboard::GetBoolean("BaseLine-Only", !!0);

        if(left){
         fieldPos = 0;
        } else if(center){
            fieldPos = 1;
        } else if(right){
            fieldPos = 2;
        } else {
            fieldPos = -1;
        }
		string position;
		position = inputData;
		//switchPosition indicates where the switch is located, 0 = left, 1 = right.
		int switchposition_cl;
        int switchposition_fr;
        int scaleposition;
		//String parsing code.
		if(position == "LLL") {
			LLL = true;
		} else if(position == "RRR") {
			RRR = true;
		} else if(position == "LRL") {
			LRL = true;
		} else if(position == "RLR") {
			RLR = true;
		}

		//Assigns 0 to the left side, 1 to the right side.
		if (LLL) {
			switchposition_cl = 0;
            scaleposition = 0;
            switchposition_fr = 0;
		} else if (RRR) {
			switchposition_cl = 1;
            scaleposition = 1;
            switchposition_fr = 1;
		} else if (LRL) {
			switchposition_cl = 0;
            scaleposition = 1;
            switchposition_fr = 0;
		} else if (RLR) {
			switchposition_cl = 1;
            scaleposition = 0;
            switchposition_fr = 1;
		} else {
			switchposition_cl = 2;
            scaleposition = 2;
            switchposition_fr = 2;
		}
        if(runSwitch) {
			// Should be accurate to some degree Without any testing
			//TODO TEST!
            switch (switchposition_cl) {
                case LEFT_FIELD:
                    switch (fieldPos) {
                        case BOT_ON_LEFT:
                            Segments[0].DIST = 168.0;
                            Segments[0].ANGLE = 90.0;
                            Segments[1].DIST = 4.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = 0.0;
                            break;
                        case BOT_ON_CENTER:
                            Segments[0].DIST = 95.0;
                            Segments[0].ANGLE = -90.0;
                            Segments[1].DIST = 115.0;
                            Segments[1].ANGLE = -90.0;
                            Segments[2].DIST = 97.0;
                            Segments[2].ANGLE = 90.0;
							Segments[3].DIST = 30.0;
							Segments[3].ANGLE = 0.0;
                            break;
                        case BOT_ON_RIGHT:
                            Segments[0].DIST = 95.0;
                            Segments[0].ANGLE = -90.0;
                            Segments[1].DIST = 240.0;
                            Segments[1].ANGLE = 90.0;
                            Segments[2].DIST = 95.0;
                            Segments[2].ANGLE = 90.0;
							Segments[3].DIST = 20.0;
							Segments[3].ANGLE = 0.0;
                            break;
                        case BOT_POS_FAILURE:
                            Segments[0].DIST = 0.0;
                            Segments[0].ANGLE = 0.0;
                            Segments[1].DIST = 0.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = 0.0;
                            break;
                    }
                    break;
                case RIGHT_FIELD:
                    switch (fieldPos) {
                        case BOT_ON_LEFT:
							Segments[0].DIST = 95.0;
							Segments[0].ANGLE = 90.0;
							Segments[1].DIST = 240.0;
							Segments[1].ANGLE = -90.0;
							Segments[2].DIST = 60.0;
							Segments[2].ANGLE = -90.0;
							Segments[3].DIST = 20.0;
							Segments[3].ANGLE = 0.0;
                            break;
                        case BOT_ON_CENTER:
							Segments[0].DIST = 95.0;
							Segments[0].ANGLE = 90.0;
							Segments[1].DIST = 115.0;
							Segments[1].ANGLE = -90.0;
							Segments[2].DIST = 95.0;
							Segments[2].ANGLE = -90.0;
							Segments[3].DIST = 20.0;
							Segments[3].ANGLE = 0.0;
                            break;
                        case BOT_ON_RIGHT:
                            Segments[0].DIST = 95.0;
                            Segments[0].ANGLE = 90.0;
                            Segments[1].DIST = 36.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = Segments[1].ANGLE;
                            break;
                        case BOT_POS_FAILURE:
                            Segments[0].DIST = 0.0;
                            Segments[0].ANGLE = 0.0;
                            Segments[1].DIST = 0.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = 0.0;
                            break;
                    }
                    break;
                case FIELD_FAILURE:
                    Segments[0].DIST = 0.0;
                    Segments[0].ANGLE = 0.0;
                    Segments[1].DIST = 0.0;
                    Segments[1].ANGLE = 0.0;
                    Segments[2].DIST = 0.0;
                    Segments[2].ANGLE = 0.0;
                    break;
            }
        }
        if(runScale){
            //TODO Make run scale program
            switch (scaleposition) {
                case LEFT_FIELD:
                    switch (fieldPos) {
                        case BOT_ON_LEFT:
                            Segments[0].DIST = 317.0; //263 +54
                            Segments[0].ANGLE = 90.0;
                            Segments[1].DIST = 0.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = Segments[1].ANGLE;
							// Rise elevator to Top, eject cube.
                            break;
                        case BOT_ON_CENTER:
                            Segments[0].DIST = 95.0;
                            Segments[0].ANGLE = -90.0;
                            Segments[1].DIST = 36.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = Segments[1].ANGLE;
                            break;
                        case BOT_ON_RIGHT:
                            Segments[0].DIST = 95.0;
                            Segments[0].ANGLE = -90.0;
                            Segments[1].DIST = 48.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = Segments[1].ANGLE;
                            break;
                        case BOT_POS_FAILURE:
                            Segments[0].DIST = 0.0;
                            Segments[0].ANGLE = 0.0;
                            Segments[1].DIST = 0.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = 0.0;
                            break;
                    }
                    break;
                case RIGHT_FIELD:
                    switch (fieldPos) {
                        case BOT_ON_LEFT:
                            Segments[0].DIST = 95.0;
                            Segments[0].ANGLE = 90.0;
                            Segments[1].DIST = 12.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = Segments[1].ANGLE;
                            break;
                        case BOT_ON_CENTER:
                            Segments[0].DIST = 95.0;
                            Segments[0].ANGLE = 90.0;
                            Segments[1].DIST = 24.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = Segments[1].ANGLE;
                            break;
                        case BOT_ON_RIGHT:
							Segments[0].DIST = 95.0;
							Segments[0].ANGLE = -90.0;
							Segments[1].DIST = 24.0;
							Segments[1].ANGLE = 0.0;
							Segments[2].DIST = 0.0;
							Segments[2].ANGLE = Segments[1].ANGLE;
                            break;
                        case BOT_POS_FAILURE:
                            Segments[0].DIST = 0.0;
                            Segments[0].ANGLE = 0.0;
                            Segments[1].DIST = 0.0;
                            Segments[1].ANGLE = 0.0;
                            Segments[2].DIST = 0.0;
                            Segments[2].ANGLE = 0.0;
                            break;
                    }
                    break;
                case FIELD_FAILURE:
                    Segments[0].DIST = 0.0;
                    Segments[0].ANGLE = 0.0;
                    Segments[1].DIST = 0.0;
                    Segments[1].ANGLE = 0.0;
                    Segments[2].DIST = 0.0;
                    Segments[2].ANGLE = 0.0;
                    break;
            }
        }
	}
	void TURN_PARAM_SET() {
		//This function, and DRIVE_PARAM_SET, should set up the PID values once in the autonomous periodic code.
		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.5, 10);
		leftMasterMotor->Config_kI(0, 0.001, 10);
		leftMasterMotor->Config_kD(0, 0.050, 10);

		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.5, 10);
		rightMasterMotor->Config_kI(0, 0.001, 10);
		rightMasterMotor->Config_kD(0, 0.050, 10);
	}
	void DRIVE_PARAM_SET() {
		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.0085, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.50, 10);

		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.0075, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.50, 10);
	}
	void SHIFT_HIGH () {
		gearBox->Set(DoubleSolenoid::kForward);
		Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
		//Where green is, and red is.
		isInHighGear = true;
		SmartDashboard::PutBoolean("Gear State", isInHighGear);
	}
	void SHIFT_LOW () {
		gearBox->Set(DoubleSolenoid::kReverse);
		Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
		isInHighGear = false;
		SmartDashboard::PutBoolean("Gear State", isInHighGear);
	}
	void INTAKE_ARM_IN() {
		intakeArmActuator->Set(DoubleSolenoid::kForward);
		Wait(0.5);
		intakeArmActuator->Set(DoubleSolenoid::kOff);
		isActuator =! isActuator;
		//Where green is, and red is.
		isArmIn = true;
		SmartDashboard::PutBoolean("Actuator State", isArmIn);
	}
	void INTAKE_ARM_OUT() {
		intakeArmActuator->Set(DoubleSolenoid::kReverse);
		Wait(0.5);
		intakeArmActuator->Set(DoubleSolenoid::kOff);
		isActuator =! isActuator;
		isArmIn = false;
		SmartDashboard::PutBoolean("Actuator State", isArmIn);
	}
	void AutonomousInit() override {
		SEGMENT_DATA_SET(DriverStation::GetInstance().GetGameSpecificMessage());
		SHIFT_HIGH();

		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.0085, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.50, 10);

		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.0075, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.50, 10);

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->Set(ControlMode::Position, 0);
		rightMasterMotor->Set(ControlMode::Position, 0);
		rightMasterMotor->SetInverted(true);
		rightSlaveMotor->SetInverted(true);
		rightMasterMotor->SetSensorPhase(true);

		autonomousTimer->Reset();
		autonomousTimer->Start();
		NAVXBoard->Reset();
		ADXGyro->Reset();

		driveState = SEG_1;
		doneDriving = false;
		doneTurning = false;
	}

	void AutonomousPeriodic() {

		//Baseline autonomous, aim the robot and let it go.
		if(autonomousTimer->Get() < 4.5) {
			leftMasterMotor->Set(ControlMode::PercentOutput, -0.5);
			rightMasterMotor->Set(ControlMode::PercentOutput, -0.5);
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		}
		//Basic center position cube autonomous.
		if(autonomousTimer->Get() < 1.0) {
			actuatorMotor->Set(ControlMode::PercentOutput, -0.5);
		} else if(autonomousTimer->Get() < 2.0) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.0);
			elevatorMasterMotor->Set(ControlMode::Position, 18000);
		} else if(autonomousTimer->Get() < 15.0) {
			if(!doneDriving) {
				//Perhaps use time and motor drive percent instead of drive function?
				doneDriving = DRIVE_TO_DISTANCE(95);
			} else {
				leftMasterMotor->Set(ControlMode::PercentOutput, 0);
				rightMasterMotor->Set(ControlMode::PercentOutput, 0);
				intakeMasterMotor->Set(ControlMode::PercentOutput, 0.5);
			}
		}
		//Complex autonomous program, modular, procedural for baseline/switch/scale.
		//Requires the segment code to be functional.
		//Repurposes the bag box and elevator code in the opening seconds.
		if(autonomousTimer->Get() < 1.0) {
			actuatorMotor->Set(ControlMode::PercentOutput, -0.5);
		} else if(autonomousTimer->Get() < 2.0) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.0);
			elevatorMasterMotor->Set(ControlMode::Position, 18000);
		} else if(autonomousTimer->Get() < 14.0) {

		} else if(autonomousTimer->Get() < 15.0) {

		} else {

		}
	}

//			if(!doneDriving) {
//				if(drivingParamTest) {
//					DRIVE_PARAM_SET();
//					Wait(0.5);
//					leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
//					rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
//					drivingParamTest = false;
//				} else if(!drivingParamTest) {
//					doneDriving = DRIVE_TO_DISTANCE(130);
//				}
//			} else {
//				leftMasterMotor->Set(ControlMode::PercentOutput, 0);
//				rightMasterMotor->Set(ControlMode::PercentOutput, 0);
//			}
//			} else if(doneDriving && !doneTurning){
//				if(turningParamTest) {
//					TURN_PARAM_SET();
//					Wait(0.5);
//					turningParamTest = false;
//				} else if (!turningParamTest) {
//					doneTurning = TURN_TO_ANGLE(Segments[driveState].ANGLE);
//				}
//			}
//				doneTurning = TURN_TO_ANGLE(90);
//			} else if (doneDriving && doneTurning) {
//				elevatorMasterMotor->Set(ControlMode::Position, 18000);
//			}
//			if(autonomousTimer->Get() > 14.0) {
//				intakeMasterMotor->Set(ControlMode::PercentOutput, -0.5);
//			}
//
//		//Selection based on switch position, or robotic position.
//		//isTracking will need to be retrieved from networktables.
//		//Time and Ultrasonic Distance will need to be sent to Jeff.
//		if(autonomousTimer->Get() <= 0.75) {
//			actuatorMotor->Set(ControlMode::PercentOutput, 0.25);
//		} else if (autonomousTimer->Get() <= 1.5) {
//			elevatorMasterMotor->Set(ControlMode::Position, 2000);
//		} else if (autonomousTimer->Get() <= 15.0) {
//			elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
//			elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
//			if(driveState <= 5) {
//				if(!doneDriving) {
//					//The below state checker should only set the turn PID parameters once only.
//					if(drivingParamTest) {
//						DRIVE_PARAM_SET();
//						Wait(0.5);
//						leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
//						rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
//						drivingParamTest = false;
//					} else if(!drivingParamTest) {
//						doneDriving = DRIVE_TO_DISTANCE(Segments[driveState].DIST);
//						SmartDashboard::PutNumber("Current Distance", Segments[driveState].DIST);
//						SmartDashboard::PutNumber("Current Angle", Segments[driveState].ANGLE);
//						SmartDashboard::PutString("Control State", "Moving Forward!");
//						SmartDashboard::PutString("Turning State?", "Not Turning!");
//						SmartDashboard::PutBoolean("Driving State", doneDriving);
//						SmartDashboard::PutBoolean("Turning State", driveState);
//					}
//				} else if (doneDriving && !doneTurning) {
//					if(turningParamTest) {
//						TURN_PARAM_SET();
//						Wait(0.5);
//						turningParamTest = false;
//					} else if (!turningParamTest) {
//						doneTurning = TURN_TO_ANGLE(Segments[driveState].ANGLE);
//						SmartDashboard::PutNumber("Current Distance", Segments[driveState].DIST);
//						SmartDashboard::PutNumber("Current Angle", Segments[driveState].ANGLE);
//						SmartDashboard::PutString("Control State", "Not Moving Forward!");
//						SmartDashboard::PutString("Control State", "Turning!");
//						SmartDashboard::PutBoolean("Driving State", doneDriving);
//						SmartDashboard::PutBoolean("Turning State", driveState);
//					}
//				} else if(doneDriving && doneTurning) {
//					driveState ++;
//					leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
//					rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
//					SmartDashboard::PutNumber("Completed Segment:", driveState);
//					SmartDashboard::PutBoolean("Driving State", doneDriving);
//					SmartDashboard::PutBoolean("Turning State", driveState);
//					turningParamTest = true;
//					drivingParamTest = true;
//					doneDriving = false;
//					doneTurning = false;
//				}
//			} else if(driveState > 5) {
//					leftMasterMotor->Set(ControlMode::PercentOutput, 0);
//					rightMasterMotor->Set(ControlMode::PercentOutput, 0);
//					if(autonomousTimer->Get() < 14.0) {
//						elevatorMasterMotor->Set(ControlMode::Position, 2000);
//					} else if(autonomousTimer->Get() < 15.0) {
//						intakeMasterMotor->Set(ControlMode::PercentOutput, -0.5);
//
//					}
//			}
//		}

	void TeleopInit() {
		autonomousTimer->Stop();
		NAVXBoard->Reset();
		ADXGyro->Reset();

		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
		//This function should always put us into low gear, regardless of previous gear setting.
		if(!isHighGear){
			SHIFT_LOW();
		}
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->SetSensorPhase(true);
		rightMasterMotor->SetSensorPhase(false);
		leftMasterMotor->SetInverted(false);
		leftSlaveMotor->SetInverted(false);
		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);

	}
	void TeleopPeriodic() {

	if(rightJoystick->GetRawButton(8)) {
		if(isControl) {

		} else if (!isControl) {

		}
		controlSoloTest = false;
	}
	if(!rightJoystick->GetRawButton(8)) {
		controlSoloTest = true;
	}
	//Checks if the control scheme is set to true or false.
	//True =, false =.
		if(teleopMasterSwitch) {
			if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && shiftingSoloTest) {
				if(isHighGear) {
					SHIFT_HIGH();
				} else if(!isHighGear) {
					SHIFT_LOW();
				}
				shiftingSoloTest = false;
			}
			if(!rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
				shiftingSoloTest = true;
			}
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

			//soloTest acts as a single limiter to keep the if/else loop from running indefinitely.
			//If the button is pressed, and since soloTest is true, state should run the if/else once
			if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && shiftingSoloTest) {
				if(isHighGear) {
					SHIFT_HIGH();
				} else if(!isHighGear) {
					SHIFT_LOW();
				}
				shiftingSoloTest = false;
			}
			if(!rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
				shiftingSoloTest = true;
			}
		} else if(!teleopMasterSwitch) {

			if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && shiftingSoloTest) {
				if(isHighGear) {
					SHIFT_HIGH();
				} else if(!isHighGear) {
					SHIFT_LOW();
				}
				shiftingSoloTest = false;
			}
			if(!rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
				shiftingSoloTest = true;
			}
			//Thresholds are set to 5% of the Joystick's range.
			if(abs(rightJoystick->GetY()) < 0.05) {
				robotThrottle = 0;
			} else {
				robotThrottle = rightJoystick->GetY();
			}
			if(abs(leftJoystick->GetX()) < 0.05) {
				robotSteer = 0;
			} else {
				robotSteer = leftJoystick->GetX();
			}
			leftMasterMotor->Set(ControlMode::PercentOutput, robotThrottle - robotSteer);
			rightMasterMotor->Set(ControlMode::PercentOutput, -robotThrottle - robotSteer);

			//soloTest acts as a single limiter to keep the if/else loop from running indefinitely.
			//If the button is pressed, and since soloTest is true, state should run the if/else once
			if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && shiftingSoloTest) {
				if(isHighGear) {
					SHIFT_HIGH();
				} else if(!isHighGear) {
					SHIFT_LOW();
				}
				shiftingSoloTest = false;
			}
			if(!rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
				shiftingSoloTest = true;
			}
		}
		//Toggle code toggling the actuator
		if(XboxGamepad->GetRawButton(XboxGamepadMap::GAMEPAD_ACTUATOR_TOGGLE) && actuatorSoloTest) {
			if(isActuator) {
				INTAKE_ARM_OUT();
			} else if(!isActuator) {
				INTAKE_ARM_IN();
			}
			actuatorSoloTest = false;
		}
		if(!XboxGamepad->GetRawButton(XboxGamepadMap::GAMEPAD_ACTUATOR_TOGGLE)) {
			actuatorSoloTest = true;
		}

		if(XboxGamepad->GetPOV() == XboxGamepadMap::JOYSTICK_ACTUATOR_DOWN) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.5);
		} else if(XboxGamepad->GetPOV() == XboxGamepadMap::JOYSTICK_ACTUATOR_UP) {
			actuatorMotor->Set(ControlMode::PercentOutput, -0.5);
		} else {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.0);
		}

		elevatorThrottle = XboxGamepad->GetRawAxis(XboxGamepadMap::ELEVATOR_AXIS);
		elevatorMasterMotor->Set(ControlMode::PercentOutput, -elevatorThrottle);

		intakeThrottle = XboxGamepad->GetRawAxis(XboxGamepadMap::INTAKE_AXIS);
		intakeMasterMotor->Set(ControlMode::PercentOutput, intakeThrottle);

		if((elevatorMasterMotor->GetSelectedSensorPosition(0) > 0) && (elevatorMasterMotor->GetSelectedSensorPosition(0) < 1000)) {
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
		}
		if((elevatorMasterMotor->GetSelectedSensorPosition(0) > 50000) && (elevatorMasterMotor->GetSelectedSensorPosition(0) < 50000)) {
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
		}
/*
		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);
*/
		SmartDashboard::PutNumber("Intake Throttle", intakeThrottle);
		SmartDashboard::PutNumber("Elevator Throttle", elevatorThrottle);

		SmartDashboard::PutNumber("Gamepad Left Y", XboxGamepad->GetRawAxis(1));
		SmartDashboard::PutNumber("Gamepad Right Y", XboxGamepad->GetRawAxis(5));

		SmartDashboard::PutNumber("Elevator Master Motor Output", elevatorMasterMotor->GetMotorOutputPercent());
		SmartDashboard::PutNumber("Elevator Slave Motor Output", elevatorSlaveMotor->GetMotorOutputPercent());
		SmartDashboard::PutNumber("Intake Master Motor Output", intakeMasterMotor->GetMotorOutputPercent());
		SmartDashboard::PutNumber("Intake Slave Motor Output", intakeSlaveMotor->GetMotorOutputPercent());
		SmartDashboard::PutNumber("Bag Box Output", actuatorMotor->GetMotorOutputPercent());

		SmartDashboard::PutNumber("Elevator Position", elevatorMasterMotor->GetSelectedSensorPosition(0));

		frc::Wait(0.005);
	}
	void TestPeriodic() {

	}
private:
};
START_ROBOT_CLASS(Robot)
