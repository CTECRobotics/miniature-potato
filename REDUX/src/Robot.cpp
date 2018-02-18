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
	//WHO TOUCH MA S P A G H E T.
	//Creates three Leg_Data structs as writable segments for use in autonomous control.
	//HAIL HYDRA.
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
	//Creates enum values based on robot positioning for use in autonomous.
	enum ROBOT_STARTING_POS{
		BOT_ON_LEFT = 0,
		BOT_ON_CENTER = 1,
		BOT_ON_RIGHT = 2,
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
	enum gamepadMap{
		ELEVATOR_CONTROL = 2,
		INTAKE_CONTROL = 5,
	};
	//Creates enum values for my behavior states.
	enum ANGEL_STATES{
		NORMAL = 0,
		CHAOTIC_NEUTRAL = 1,
		IOSIF_STALIN = 2,
		HAMMER_TIME = 3,
	};
	bool isTracking;
	bool isHighGear;
	bool soloTest;
	bool targetReached;
	bool doneDriving;
	bool doneTurning;
	int autoMode;
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
	double elevatorThrottle;
	//Variable for use in the TURN_TO_ANGLE function as the setpoint.
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
	TalonSRX *winchMasterMotor;
	TalonSRX *winchSlaveMotor;
	TalonSRX *intakeMasterMotor;
	TalonSRX *intakeSlaveMotor;
	TalonSRX *actuatorMotor;
	TalonSRX *theLonelyMotor;
	//DoubleSolenoid double actuating valve for shifting mechanism.
	DoubleSolenoid *gearBox;
	//DoubleSolenoid double actuating valve for cube grabbing mechanism.
	DoubleSolenoid *armActuator;
	//Analog ultrasonic sensor plugged into the Roborio.
	AnalogInput *roborioUltrasonic;
	//The SPI port gyroscope, designated by name.
	ADXRS450_Gyro *ADXGyro;
	//NAVX sensor and navigation board.
	AHRS *NAVXBoard;
	Timer *autonomousTimer;
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	Joystick *unamedGamepad;
	//Creation of NetworkTableEntries for entries in the NetworkTable.
	NetworkTableEntry cameraErrorAngle;
	NetworkTableEntry trackingState;
	NetworkTableEntry ultrasonicDistance;

	void RobotInit() {
		isTracking = false;
		soloTest = true;
		isHighGear = false;
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
		robotSteer = 0;
		autonomousAngleSet = 0;
		autonomousRange= 0;
		allianceColor = "";

		leftMasterMotor = new TalonSRX(1);
		leftSlaveMotor = new TalonSRX(2);
		rightMasterMotor = new TalonSRX(3);
		rightSlaveMotor = new TalonSRX(4);
		elevatorMasterMotor = new TalonSRX(5);
		elevatorSlaveMotor = new TalonSRX(6);
		intakeMasterMotor = new TalonSRX(7);
		intakeSlaveMotor = new TalonSRX(8);
		winchMasterMotor = new TalonSRX(9);
		winchSlaveMotor = new TalonSRX(10);
		actuatorMotor = new TalonSRX(11);
		theLonelyMotor = new TalonSRX(12);

		gearBox = new DoubleSolenoid(1, 2);
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
		unamedGamepad = new Joystick(2);
		ADXGyro = new ADXRS450_Gyro();
		roborioUltrasonic = new AnalogInput(0);
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
		winchMasterMotor->Set(ControlMode::PercentOutput, 0);
		winchSlaveMotor->Set(ControlMode::Follower, 9);

		actuatorMotor->Set(ControlMode::PercentOutput, 0);
			//INTERNAL PID SETUP

		//The pidxid of the sensor can be found off the configuration of the Roborio.
		//Should be a 0, multiple id values not supported (yet).
		//Selects the sensor type and the channel the sensor is communicating on.
		leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		//Sets up the sensor type, the sensor ID number, DEFINED AS pidIdx, and timeout period.
		//Sets the read direction on the Encoders, with Clockwise being positive.
		leftMasterMotor->SetSensorPhase(true);
		//Sets up the various PIDF values to tune the motor output.
		//P = (Desired motor output percentage * 1023)/(error).
		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.0075, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.50, 10);
		leftMasterMotor->ConfigMotionCruiseVelocity(2048, 10);
		leftMasterMotor->ConfigMotionAcceleration(1024, 10);

		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
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

		elevatorMasterMotor->ConfigForwardSoftLimitThreshold(10000, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitThreshold(-10000, 10);
		//Setting up of soft limits, essentially a min/max movement range for the elevator mechanism.
		//The units are defined in CTRE's proprietary units, as in 4096 units per rotation.
		elevatorMasterMotor->ConfigForwardSoftLimitEnable(true, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitEnable(true, 10);

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		//Sets up the NetworkTables for the
		auto table = NetworkTableInstance::GetDefault();
		auto networkTableData = table.GetTable("Jetson");
		cameraErrorAngle = networkTableData->GetEntry("angle");
		trackingState = networkTableData->GetEntry("V_Working");
		ultrasonicDistance = networkTableData->GetEntry("Distance");
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
		if(((roborioUltrasonic->GetAverageValue() * masterUltrasonicConversion) > 10.0) &&
				(((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick) &&
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
	void SEGMENT_SELECTION (double processedData) {
		int switchposition = processedData;
		//TODO Write stuff to get a value from the DriverStation.
		fieldPos = 2;
		//switchposition is the variable determining switch position, left or right.
		//Switch on the left side.
		switch (switchposition) {
		case LEFT_FIELD:
			switch (fieldPos) {
			//fieldPos is the variable determining robot position on the field , left, center, middle.
			case BOT_ON_LEFT:
				//Switch and Case Values have no meaning, yet.
				Segments[0]->DIST = 95.0;
				Segments[0]->ANGLE = -90.0;
				Segments[1]->DIST = 24.0;
				Segments[1]->ANGLE = 0.0;
				Segments[2]->DIST = 0.0;
				Segments[2]->ANGLE = Segments[1]->ANGLE;
				break;
			case BOT_ON_CENTER:
				Segments[0]->DIST = 95.0;
				Segments[0]->ANGLE = -90.0;
				Segments[1]->DIST = 36.0;
				Segments[1]->ANGLE = 0.0;
				Segments[2]->DIST = 0.0;
				Segments[2]->ANGLE = Segments[1]->ANGLE;
				break;
			case BOT_ON_RIGHT:
				Segments[0]->DIST = 95.0;
				Segments[0]->ANGLE = -90.0;
				Segments[1]->DIST = 48.0;
				Segments[1]->ANGLE = 0.0;
				Segments[2]->DIST = 0.0;
				Segments[2]->ANGLE = Segments[1]->ANGLE;
				break;
			}
			break;
			//Switch on the right side.
		case RIGHT_FIELD:
			switch (fieldPos) {
			case BOT_ON_LEFT:
				Segments[0]->DIST = 95.0;
				Segments[0]->ANGLE = 90.0;
				Segments[1]->DIST = 12.0;
				Segments[1]->ANGLE = 0.0;
				Segments[2]->DIST = 0.0;
				Segments[2]->ANGLE = Segments[1]->ANGLE;
				break;
			case BOT_ON_CENTER:
				Segments[0]->DIST = 95.0;
				Segments[0]->ANGLE = 90.0;
				Segments[1]->DIST = 24.0;
				Segments[1]->ANGLE = 0.0;
				Segments[2]->DIST = 0.0;
				Segments[2]->ANGLE = Segments[1]->ANGLE;
				break;
			case BOT_ON_RIGHT:
				Segments[0]->DIST = 95.0;
				Segments[0]->ANGLE = 90.0;
				Segments[1]->DIST = 36.0;
				Segments[1]->ANGLE = 0.0;
				Segments[2]->DIST = 0.0;
				Segments[2]->ANGLE = Segments[1]->ANGLE;
				break;
			}
			break;
		}

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
	float ULTRASONIC_CONVERSION(float distance) {
		float masterUltrasonicConversion = (5000.0 * 39.4)/4.88;
		float ultrasonicDistance = distance;
		ultrasonicDistance = masterUltrasonicConversion * roborioUltrasonic->GetAverageValue();
		return ultrasonicDistance;
		SmartDashboard::PutNumber("Ultrasonic Distance, Inches", ultrasonicDistance);
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

		SEGMENT_SELECTION(DriverStation::GetInstance().GetGameSpecificMessage());
		driveState = SEG_1;
	}

	void AutonomousPeriodic() {
		//Selection based on switch position, or robotic position.
		//isTracking will need to be retrieved from networktables.
		//Time and Ultrasonic Distance will need to be sent to Jeff.
		autonomousRange = roborioUltrasonic->GetAverageValue();
		float angle = cameraErrorAngle.GetDouble(Segments[driveState]->ANGLE);
		int isTracking = trackingState.GetDouble(0.0);
		ultrasonicDistance.SetDouble(double(ULTRASONIC_CONVERSION(roborioUltrasonic->GetAverageValue())));
		while(autonomousTimer->Get() <= 15) {
			if(isTracking == 0) {
				if(!doneDriving) {
				doneDriving = DRIVE_TO_DISTANCE(Segments[driveState]->DIST);
				SmartDashboard::PutString("Control State", "Moving Foward!");
				SmartDashboard::PutBoolean("Driving State", doneDriving);
				SmartDashboard::PutBoolean("Turning State", driveState);
				} else if (doneDriving) {
					doneTurning = TURN_TO_ANGLE(Segments[driveState]->ANGLE);
					SmartDashboard::PutString("Control State", "Turning!");
					SmartDashboard::PutBoolean("Driving State", doneDriving);
					SmartDashboard::PutBoolean("Turning State", driveState);
				}
				if(doneDriving && doneTurning) {
					driveState ++;
					doneDriving = false;
					doneTurning = false;
					SmartDashboard::PutString("Completed Segment:", "driveState");
					SmartDashboard::PutBoolean("Driving State", doneDriving);
					SmartDashboard::PutBoolean("Turning State", driveState);
				}
			} else if(isTracking == 1) {
				if(angle < 10) {
					angle += 10;
				}
				doneTurning = TURN_TO_ANGLE(angle);
				SmartDashboard::PutNumber("Angle Error", angle);
				SmartDashboard::PutBoolean("Driving State", doneDriving);
				SmartDashboard::PutBoolean("Turning State", driveState);
				if(doneTurning) {
					doneDriving = DRIVE_TO_DISTANCE((masterUltrasonicConversion * roborioUltrasonic->GetAverageValue()) - 18.0);
				} else if(doneTurning && doneDriving) {
					elevatorMasterMotor->Set(ControlMode::Position, 2048.0);
				}
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
		leftMasterMotor->SetSensorPhase(true);
		rightMasterMotor->SetSensorPhase(false);
		leftMasterMotor->SetInverted(false);
		leftSlaveMotor->SetInverted(false);
		rightMasterMotor->SetInverted(false);
		rightSlaveMotor->SetInverted(false);

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
			elevatorMasterMotor->Set(ControlMode::PercentOutput, 0.75);
		}
		if(rightJoystick->GetRawButton(rightJoystickMap::ELEVATOR_DOWN)) {
			elevatorMasterMotor->Set(ControlMode::PercentOutput, -0.75);
		}
		if(rightJoystick->GetRawButton(rightJoystickMap::ACTUATOR_UP)) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.625);
		}
		if(rightJoystick->GetRawButton(rightJoystickMap::ACTUATOR_DOWN)) {
			actuatorMotor->Set(ControlMode::PercentOutput, -0.25);
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
