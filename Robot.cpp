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
	//For tracking the current segment during autonomous.

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
		JOYSTICK_ACTUATOR_UP = 1,
		JOYSTICK_ACTUATOR_DOWN = 5,
		INTAKE = 5,
		OUTAKE = 6,
	};
	enum gamepadMap{
		ARM_PISTON_IN = 1,
		ARM_PISTON_OUT= 2,
		GAMEPAD_ACTUATOR_DOWN = 1,
		GAMEPAD_ACTUATOR_UP = 5,
	};
	bool isTracking;
	bool isHighGear;
	bool isActuator;
	bool isTurnState;
	bool isDriveState;
	bool isArmOut;
	bool isArmIn;
	bool shiftingSoloTest;
	bool actuatorSoloTest;
	bool turningParamTest;
	bool drivingParamTest;
	bool targetReached;
	bool doneDriving;
	bool doneTurning;
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
	double elevatorGamepadThrottle;
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
	//Proprietary unit for each Encoder rotation, 4096 units per rotation.
	const float encoderRotTick = 4096.0;
	const float PI = 3.1415;
	//Calculation of circumference, used for autonomous driving control.
	const float circumference = (PI*6.0);
	//Calculation of converting returned ultrasonic sensor voltage to meaningful value.
	const float masterUltrasonicConversion = (5000.0 * 39.4)/4.88;
	//Ratio of gears from Encoder to actual driveshaft.
	const float gearRatio = (2.0/15.0);
	const float motorVelocity = 2048.0;
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
	Joystick *PS4Gamepad;
	//Creation of NetworkTableEntries for entries in the NetworkTable.
	//Note the lack of a *, this is because of interaction between pointer and NetworkTableEntry.
	//TODO Renable these NetorkTableEntry objects later after Clackamas.
//	NetworkTableEntry cameraErrorAngle;
//	NetworkTableEntry trackingState;
//	NetworkTableEntry ultrasonicDistance;
//	NetworkTableEntry dataSink;
//	NetworkTableEntry scaleString;
	void RobotInit() {
		isTracking = false;
		isHighGear = false;
		isActuator = false;
		isTurnState = true;
		isDriveState = true;
		shiftingSoloTest = true;
		actuatorSoloTest = true;
		turningParamTest = true;
		drivingParamTest = true;
		targetReached = false;
		doneDriving = false;
		doneTurning = false;
		autoMode = 0;
		fieldPos = 0;
		gyroSetpoint = 0;
		navxGyro = 0;
		rioGyro = 0;
		combinedGyroValue = 0;
		robotThrottle = 0;
		elevatorJoystickThrottle = 0;
		elevatorGamepadThrottle = 0;
		intakeThrottle = 0;
		robotSteer = 0;
		autonomousAngleSet = 0;
		autonomousRange= 0;

		leftMasterMotor = new TalonSRX(1);
		leftSlaveMotor = new TalonSRX(2);
		rightMasterMotor = new TalonSRX(3);
		rightSlaveMotor = new TalonSRX(4);

		elevatorMasterMotor = new TalonSRX(5);
		elevatorSlaveMotor = new TalonSRX(6);

		intakeMasterMotor = new TalonSRX(7);
		intakeSlaveMotor = new TalonSRX(8);

		actuatorMotor = new TalonSRX(9);

		gearBox = new DoubleSolenoid(1, 2);
		intakeArmActuator = new DoubleSolenoid(3, 4);
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
		PS4Gamepad = new Joystick(0);
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
		intakeSlaveMotor->Set(ControlMode::Follower, 7);
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
		elevatorMasterMotor->SetSensorPhase(true);
		elevatorMasterMotor->Config_kF(0, 0.0, 10);
		elevatorMasterMotor->Config_kP(0, 0.0075, 10);
		elevatorMasterMotor->Config_kI(0, 0.0, 10);
		elevatorMasterMotor->Config_kD(0, 0.50, 10);

		elevatorMasterMotor->ConfigForwardSoftLimitThreshold(10000, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitThreshold(-10000, 10);
		//Setting up of soft limits, essentially a min/max movement range for the elevator mechanism.
		//The units are defined in CTRE's proprietary units, as in 4096 units per rotation.
		elevatorMasterMotor->ConfigForwardSoftLimitEnable(true, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitEnable(true, 10);

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		//Ensure that the elevator is set to the LOWEST position on the linear slide.
		//Additionally, ensure that there is no slack in the cables.
		elevatorMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		//Sets up the NetworkTables for communication.
		//Grabs the default NetworkTable the roboRio creates.
		//TODO Renable NetworkTable code after Clackamas.
//		auto table = NetworkTableInstance::GetDefault();
//		//Creates a NetworkTable called Jetson based on the default NetworkTable.
//		auto networkTableData = table.GetTable("Jetson");
//		//Entry with the calculated error from the TX1.
//		cameraErrorAngle = networkTableData->GetEntry("angle");
//		//Entry with the state of the vision tracking, 1 = tracking, 0 = not.
//		trackingState = networkTableData->GetEntry("V_Working");
//		//Entry with the ultrasonic distance.
//		ultrasonicDistance = networkTableData->GetEntry("Distance");
//		//Entry with the switch/scale position string.
//		//Retrieved from the DriverStation.
//		scaleString = networkTableData->GetEntry("scaleString");
//
//		dataSink = networkTableData->GetEntry("processed");
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
		double motorVelocity = 2048;
		double tolerance = 9.0;

		if((!targetReached && (combinedGyroValue < autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::Velocity, (-motorVelocity));
			rightMasterMotor->Set(ControlMode::Velocity, (-motorVelocity));
			doneTurning = false;
			if((combinedGyroValue > autonomousAngleSet - tolerance) && (
					combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			SmartDashboard::PutString("Turning State?", "Turning Right!");

		} else if(!targetReached && ((combinedGyroValue > autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::Velocity, (motorVelocity));
			rightMasterMotor->Set(ControlMode::Velocity, (motorVelocity));
			doneTurning = false;
			if((combinedGyroValue > autonomousAngleSet - tolerance) &&
					(combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			SmartDashboard::PutString("Turning State?", "Turning Left!");

		}
		if(targetReached){
			leftMasterMotor->Set(ControlMode::PercentOutput, (0));
			rightMasterMotor->Set(ControlMode::PercentOutput, (0));
			doneTurning = true;
			targetReached = true;
			SmartDashboard::PutString("Turning State?", "Done!");
		}
return doneTurning;
	}
	bool DRIVE_TO_DISTANCE(int autonomousDistanceSet) {

		bool doneDriving;
		rightMasterMotor->SetInverted(true);
		rightSlaveMotor->SetInverted(true);
		rightMasterMotor->SetSensorPhase(true);
		if((((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick) &&
				(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick)))) {
			//Does the calculations necessary to set a position for the motors.
			//autonomousDistanceSet/circumference = number of turns necessary.
			//Number of turns * encRotTick (4096) = position needed to drive to in native units.
			leftMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference)) * encoderRotTick);
			rightMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference)) * encoderRotTick);

			SmartDashboard::PutString("Driving State?", "Currently Moving Forward!");
			//When doneDriving is false, should continue driving along.
			doneDriving = false;
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);

			SmartDashboard::PutString("Driving State?", "Done!");
			//When doneDrving is true, should schedule the next segment.
			doneDriving = true;
		}
		return doneDriving;
	}
    bool ACTION(int autonomousActionSet){
        bool doneAction;
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
        bool runAll = SmartDashboard::GetBoolean("BaseLne-Scale&Switch", !!0);
        bool runBaseline = SmartDashboard::GetBoolean("BaseLine-Only", !!0);
        bool MultiCube = SmartDashboard::GetBoolean("MultiCube-Auto", !!0);

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

            if(MultiCube){
				// TODO Finish logic and drive to
                bool switch_2_cube = SmartDashboard::GetBoolean("2cubeSwitch",!!0);
                bool switch_and_scale_2_cube = SmartDashboard::GetBoolean("2cubeSwitch+scale",!!0);
                bool scale_2_cube = SmartDashboard::GetBoolean("2cubeScale",!!0);
                if(switch_2_cube){

                }
                if(switch_and_scale_2_cube){

                }
                if(scale_2_cube){

                }
				//TODO add more.
            }

	}
	void TURN_PARAM_SET() {
		//This function, and DRIVE_PARAM_SET, should set up the PID values once in the autonomous periodic code.
		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.5, 10);
		leftMasterMotor->Config_kI(0, 0.0001, 10);
		leftMasterMotor->Config_kD(0, 0.050, 10);

		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.5, 10);
		rightMasterMotor->Config_kI(0, 0.0001, 10);
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
		frc::Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
	}
	void SHIFT_LOW () {
		gearBox->Set(DoubleSolenoid::kReverse);
		frc::Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
	}
	void INTAKE_ARM_IN() {
		intakeArmActuator->Set(DoubleSolenoid::kForward);
		frc::Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isActuator =! isActuator;

	}
	void INTAKE_ARM_OUT() {
		intakeArmActuator->Set(DoubleSolenoid::kReverse);
		frc::Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isActuator =! isActuator;
	}
	void AutonomousInit() override {
		//TODO Renable later.
//		scaleString.SetString(DriverStation::GetInstance().GetGameSpecificMessage());
		// SmartDashboard::PutString("Game Data", DriverStation::GetInstance().GetGameSpecificMessage());
		//fieldPos = SmartDashboard::GetNumber("Robot Position", 3);
		SEGMENT_DATA_SET(DriverStation::GetInstance().GetGameSpecificMessage());
		SHIFT_LOW();

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
		//TODO Change the default value back to -1.0 when finished with testing.

		driveState = SEG_1;
		doneDriving = false;
		doneTurning = false;
	}

	void AutonomousPeriodic() {
		//Selection based on switch position, or robotic position.
		//isTracking will need to be retrieved from networktables.
		//Time and Ultrasonic Distance will need to be sent to Jeff.
		//TODO Renable later.
//		float angle = cameraErrorAngle.GetDouble(Segments[driveState].ANGLE);
//		int isTracking = trackingState.GetDouble(0.0);
		if(autonomousTimer->Get() <= 1.0) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.5);
		} else if (autonomousTimer->Get() <= 2.0) {
			elevatorMasterMotor->Set(ControlMode::Position, 2000);
		} else if (autonomousTimer->Get() <= 15.0) {
			if(driveState <= 5) {
				if(!doneDriving) {
					//The below state checker should only set the turn PID parameters once only.
					if(drivingParamTest) {
						DRIVE_PARAM_SET();
						frc::Wait(0.5);
						leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
						rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
						drivingParamTest = false;
					}
						doneDriving = DRIVE_TO_DISTANCE(Segments[driveState].DIST);
						SmartDashboard::PutNumber("Current Distance", Segments[driveState].DIST);
						SmartDashboard::PutNumber("Current Angle", Segments[driveState].ANGLE);
						SmartDashboard::PutString("Control State", "Moving Forward!");
						SmartDashboard::PutString("Turning State?", "Not Turning!");
						SmartDashboard::PutBoolean("Driving State", doneDriving);
						SmartDashboard::PutBoolean("Turning State", driveState);
				} else if (doneDriving && !doneTurning) {
					if(turningParamTest) {
						TURN_PARAM_SET();
						frc::Wait(0.5);
						turningParamTest = false;
					} else {
						turningParamTest = false;
					}
					doneTurning = TURN_TO_ANGLE(Segments[driveState].ANGLE);
					SmartDashboard::PutNumber("Current Distance", Segments[driveState].DIST);
					SmartDashboard::PutNumber("Current Angle", Segments[driveState].ANGLE);
					SmartDashboard::PutString("Control State", "Not Moving Forward!");
					SmartDashboard::PutString("Control State", "Turning!");
					SmartDashboard::PutBoolean("Driving State", doneDriving);
					SmartDashboard::PutBoolean("Turning State", driveState);
				} else if(doneDriving && doneTurning) {
					driveState ++;
					leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
					rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
					SmartDashboard::PutNumber("Completed Segment:", driveState);
					SmartDashboard::PutBoolean("Driving State", doneDriving);
					SmartDashboard::PutBoolean("Turning State", driveState);
					turningParamTest = true;
					drivingParamTest = true;
					doneDriving = false;
					doneTurning = false;
				}
					//TODO The below Vision Code will be implemented later.
	//			if(isTracking == 1) {
	//				if(angle < 0) {
	//					angle -= 10;
	//				} else {
	//					angle += 10;
	//				}
	//				doneTurning = TURN_TO_ANGLE(angle);
	//				SmartDashboard::PutNumber("Angle Error", angle);
	//				SmartDashboard::PutBoolean("Driving State", doneDriving);
	//				SmartDashboard::PutBoolean("Turning State", driveState);
	//				if(doneTurning) {
	//				} else if(doneTurning && doneDriving) {
	//					elevatorMasterMotor->Set(ControlMode::Position, 2048.0);
	//				}
	//			}
			} else if(driveState > 5) {
					leftMasterMotor->Set(ControlMode::PercentOutput, 0);
					rightMasterMotor->Set(ControlMode::PercentOutput, 0);
					// intakeMasterMotor->Set(ControlMode::PercentOutput, -1.0);
			}
		}
	}

	void TeleopInit() {
		autonomousTimer->Stop();
		NAVXBoard->Reset();
		ADXGyro->Reset();

		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);
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
		bool GamepadState;
//		double intakeDirection;
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

		intakeThrottle = PS4Gamepad->GetY();

		if(leftJoystick->GetRawButton(3)) {
			intakeThrottle = -1.0;
		}
		if(leftJoystick->GetRawButton(4)) {
			intakeThrottle = 1.0;
		}

		intakeMasterMotor->Set(ControlMode::PercentOutput, intakeThrottle);

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
		//Toggle code toggling the actuator
		if(leftJoystick->GetRawButton(rightJoystickMap::ACTUATOR_TOGGLE) && actuatorSoloTest) {
			if(isActuator) {
				INTAKE_ARM_OUT();
			} else if(!isActuator) {
				INTAKE_ARM_IN();
			}
			actuatorSoloTest = false;
		}
		if(!leftJoystick->GetRawButton(rightJoystickMap::ACTUATOR_TOGGLE)) {
			actuatorSoloTest = true;
		}
		//The gamepad does not need toggle code, only button based.
		if(PS4Gamepad->GetRawButton(gamepadMap::ARM_PISTON_IN)) {
			INTAKE_ARM_IN();
		}
		if(PS4Gamepad->GetRawButton(gamepadMap::ARM_PISTON_OUT)) {
			INTAKE_ARM_OUT();
		}
		//Actuator code for rotating the intake mechanism.
		if(leftJoystick->GetPOV() == leftJoystickMap::JOYSTICK_ACTUATOR_DOWN) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.5);
		} else if(leftJoystick->GetPOV() == leftJoystickMap::JOYSTICK_ACTUATOR_UP) {
			actuatorMotor->Set(ControlMode::PercentOutput, -0.5);
		} else {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.0);
		}
		if(PS4Gamepad->GetPOV() == gamepadMap::GAMEPAD_ACTUATOR_DOWN) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.5);
		} else if(PS4Gamepad->GetPOV() == gamepadMap::GAMEPAD_ACTUATOR_UP) {
			actuatorMotor->Set(ControlMode::PercentOutput, -0.5);
		} else {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.0);
		}

		//Master/Secondary if statements for the elevator lift.
		//Should be a complete override if the joystick moves.
		//Thresholds set to reduce "booping" errors.
		if(abs(PS4Gamepad->GetRawAxis(5)) > 0.05) {
			GamepadState = true;
		}
		if(abs(leftJoystick->GetY()) > 0.1) {
			GamepadState = false;
		}
		if(!GamepadState){
			elevatorGamepadThrottle = leftJoystick->GetY();
			elevatorMasterMotor->Set(ControlMode::PercentOutput, elevatorGamepadThrottle);
		} else if(GamepadState) {
			elevatorGamepadThrottle = PS4Gamepad->GetRawAxis(5);
			elevatorMasterMotor->Set(ControlMode::PercentOutput, elevatorJoystickThrottle);
		}

		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);

		SmartDashboard::PutNumber("Composite Gyroscope Value", combinedGyroValue);
		SmartDashboard::PutNumber("Gamepad Throttle?", PS4Gamepad->GetRawAxis(5));
		//TODO Renable later.
//		SmartDashboard::PutNumber("Vision Working State", trackingState.GetDouble(0.0));
//		SmartDashboard::PutNumber("Camera Error", cameraErrorAngle.GetDouble(180.0));
//		SmartDashboard::PutNumber("Switch Position", dataSink.GetDouble(-1));

		frc::Wait(0.005);
	}
	void TestPeriodic() {

	}
private:
};
START_ROBOT_CLASS(Robot)
