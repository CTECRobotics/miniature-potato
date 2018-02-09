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
#include <string>
using namespace std;
class Robot : public frc::IterativeRobot {
public:
	struct Leg_Data{
		int DIST;
		int ANGLE;
	}*Segments[3];
	//DESIGNATES THREE Leg_Data structs AS WRITABLE SEGMENTS FOR AUTONOMOUS CONTROL
	enum segmentState{
		SEG_1 = 0,
		SEG_2 = 1,
		SEG_3 = 2,
	};
	//TRACKING WHICH SEGMENT THE ROBOT IS EXECUTING
	enum switchpositionValues{
		LEFT_FIELD = 0,
		RIGHT_FIELD = 1,
	};
	//MAPPING JOYSTICK CONTROLS TO VARIOUS FUNCTIONS
	//FOR BOTH RIGHT AND LEFT STICK
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

	//VALUES FOR TRACKING WHICH SIDE OF THE FIELD THE SWITCH IS ON
	enum ROBOT_STARTING_POS{
		ROB_ON_LEFT = 0,
		ROB_ON_CENTER = 1,
		ROB_ON_RIGHT = 2,
	};
	//DESIGNATES CONTROL STATES FOR AUTONOMOUS SCHEDULING
		//REGULATORY VALUES
	bool isTracking;
	bool isHighGear;
	bool soloTest;
	int autoMode;
	int fieldPos;
		//CONTROL VALUES
	double cameraError;		//POSTIVE IS NEGATIVE, RIGHT IS POSITIVE
	double gyroSetpoint;
	double navxGyro;		//OUTPUT FROM THE NAVX
	double rioGyro;			//OUTPUT FROM THE SPI GYROSCOPE
	double combinedGyroValue;	//AVERAGED VALUE OF GYROSCOPES
	double leftVelocity;		//VELOCITY OF THE ENCODERS IN TICKS
	double rightVelocity;
	double elevatorPos;
	double throttle;
	double steer;
	double autonomousAngleSet;			//UNIVERSAL ANGLE TARGET
	double autonomousDistanceSet;		//UNIVERSAL DISTANCE TARGET
	double autonomousRange;
	int driveState;						//CONTROL VALUE FOR CONTROLING AUTONOMOUS DRIVING, WHEN TO DRIVE FOWARD AND TURN
	string allianceColor;
		//DON'T TOUCH ME VALUES
	const float valveWait = 0.25;				//TIME TO WAIT FOR AIR TO FLOW TO SOLENOIDS
	const float encoderRotTick = 4096.0;		//4096 "TICKS" PER ROTATION ACCORDING TO MAGNETIC ENCODER SPECS
	const float PI = 3.1415;
	const float circumference = (PI*6.0);
	const float ultrasonicConversion = 5000/4.88;	//5000 MM PER 4.88 VOLTAGE ACCORDING TO ULTRASONIC SPECS
	const float gearRatio = (2.0/15.0);

		//TEMP
		//CANTALON OBJECTS
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
		//SOLENOIDS
	DoubleSolenoid *gearBox;
		//SENSOR INPUTS
	AnalogInput *roborioUltrasonic;
	ADXRS450_Gyro *ADXGyro;			//NAMING IS ACCORDING TO THE PART NAME
	AHRS *NAVXBoard;				//NAVX MXP NAVIGATION/SENSOR BOARD
		//MISC
	Timer *autonomousTimer;
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	DriverStation *driverStation;

	shared_ptr<nt::NetworkTable> networkTableData;


	void RobotInit() {
				//VALUE SETUP
			//REGULATORY VALUES
		isTracking = false;
		autoMode = 0;
		fieldPos = 0;
			//CONTROL VALUES
		gyroSetpoint = 0;
		navxGyro = 0;
		rioGyro = 0;
		combinedGyroValue = 0;
		throttle = 0;
		steer = 0;
		autonomousAngleSet = 0;		//THIS IS THE UNIVERSAL ANGLE SETPOINT FOR AUTO
		autonomousRange= 0;
		allianceColor = "";
			//TEMP

				//PERIPHERAL AND CONTROL SETUP
			//TALONSRX
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
			//SOLENOIDS
		gearBox = new DoubleSolenoid(2, 1);
			//NAVX CONNECTION ATTEMPT
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
		ADXGyro->Calibrate();

			//INITIAL SETPOINTS, CALIB, ETC.
				//ControlMode::(mode) IS NOW USED TO DETERMINE CONTROL METHOD
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
			//MISC
				//INTERNAL PID SETUP
				//SELECTS SENSOR BASED OFF CHANNEL
				//VERFIY THE DERIVE ID (DEFINED AS pidIdx) VIA THE WEB-CONFIG PAGE, SHOULD BE EITHER 0 OR A 1
		leftMasterMotor->GetSelectedSensorVelocity(0);
				//SELECTS SENSOR BASED OFF CHANNEL
		leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
				//SETS UP SENSOR TYPE, PID SLOT (DEFINED AS pidIdx) NUMBER, TIMEOUT
		leftMasterMotor->SetSensorPhase(true);
				//DETERMINES READ directionS
		leftMasterMotor->ConfigNominalOutputForward(0, 10);
		leftMasterMotor->ConfigNominalOutputReverse(0, 10);
		leftMasterMotor->ConfigPeakOutputForward(1, 10);
		leftMasterMotor->ConfigPeakOutputReverse(-1, 10);
				//SETS UP THE VARIOUS VALUES IN THE PID SLOT
		leftMasterMotor->Config_kF(0, 0.375, 10);
		leftMasterMotor->Config_kP(0, 0.125, 10);
		leftMasterMotor->Config_kI(0, 0.05, 10);
		leftMasterMotor->Config_kD(0, 0.05, 10);

		rightMasterMotor->GetSelectedSensorVelocity(0);
		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		rightMasterMotor->SetSensorPhase(false);
		rightMasterMotor->ConfigNominalOutputForward(0, 10);
		rightMasterMotor->ConfigNominalOutputReverse(0, 10);
		rightMasterMotor->ConfigPeakOutputForward(1, 10);
		rightMasterMotor->ConfigPeakOutputReverse(-1, 10);
		rightMasterMotor->Config_kF(0, 0.375, 10);
		rightMasterMotor->Config_kP(0, 0.125, 10);
		rightMasterMotor->Config_kI(0, 0.05, 10);
		rightMasterMotor->Config_kD(0, 0.05, 10);

		elevatorMasterMotor->ConfigForwardSoftLimitThreshold(10000, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitThreshold(-10000, 10);
		//SOFT LIMITS TO ENSURE THAT ELEATOR SYSTEM DOES NOT BREAK THINGS
		//UNITS ARE IN NATIVE UNITS, 4096 PER ROTATION
		elevatorMasterMotor->ConfigForwardSoftLimitEnable(true, 10);
		elevatorMasterMotor->ConfigReverseSoftLimitEnable(true, 10);

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);


		NetworkTable::SetServerMode();
		//SERVER VS CLIENT MODE
		NetworkTable::SetIPAddress("roborio-6445-frc.local");
		//SETS THE ROBORIO IP ADDRESS FOR THE JETSON TX1
		NetworkTable::Initialize();
		//STARTS UP NETWORK TABLE
		shared_ptr<nt::NetworkTable> networkTableData = NetworkTable::GetTable("JETSON");
	}
	//INPUT A TURN ANGLE AND ROTATE THE ROBOT AS SUCH
	//OUTPUT A SUCCESS STATEMENT
	bool TURN_TO_ANGLE (int autonomousAngleSet) {
		const float inchToMeter = (1.0/39.4);
		const float drivetrainRadius = 12.5625;
		//DISTANCE FROM CENTER LINE OF ROBOT TO CENTER OF MIDDLE DRIVE WHEELS, WEST COAST STYLE
		//4096 "TICKS" PER ROTATION, AS COUNTED BY THE ENCODERS
		const float angletolerance = 2.0;
		float difference = 0.0;
		double direction;
		float distancePerWheel = 0;
		navxGyro = NAVXBoard->GetAngle() + 180;
		rioGyro = ADXGyro->GetAngle() + 180;
		combinedGyroValue = ((navxGyro + rioGyro)/2);
		bool doneTurning;

		difference = ((abs(autonomousAngleSet - combinedGyroValue)) * (PI/180));
			//ANGULAR difference IN RADIANS
		distancePerWheel = (drivetrainRadius * inchToMeter * difference);
			//DISTANCE TO TRAVEL PER WHEEL IN METRES
			//ESSENTIALLY DIST = RADIUS * THETA
		direction = ((sin((autonomousAngleSet - combinedGyroValue)))/abs(sin(combinedGyroValue - autonomousAngleSet)));

		SmartDashboard::PutNumber("Distance to Drive", distancePerWheel);
		SmartDashboard::PutNumber("Direction", direction);

		if((combinedGyroValue != (autonomousAngleSet - angletolerance)) || (combinedGyroValue != (autonomousAngleSet + angletolerance))) {
			leftMasterMotor->Set(ControlMode::Position, (distancePerWheel/(circumference * inchToMeter)) * encoderRotTick * direction);
			rightMasterMotor->Set(ControlMode::Position, (distancePerWheel/(circumference * inchToMeter)) * encoderRotTick * direction);
			doneTurning = false;
		} else {
			leftMasterMotor->Set(ControlMode::Velocity, 0.0);
			rightMasterMotor->Set(ControlMode::Velocity, 0.0);
			doneTurning = true;
		}
return doneTurning;
	}

	bool DRIVE_TO_DISTANCE(int autonomousDistanceSet) {
		const float tolerance = 0.0625;
		const float inchToMeter = (1.0/39.4);

		bool doneDriving;

		if((leftMasterMotor->GetSelectedSensorPosition(0) < (autonomousDistanceSet - tolerance)) && (rightMasterMotor->GetSelectedSensorPosition(0) < (autonomousDistanceSet - tolerance))) {
			leftMasterMotor->Set(ControlMode::Position, (autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);
			rightMasterMotor->Set(ControlMode::Position, (autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);
			doneDriving = false;
		} else {
			leftMasterMotor->Set(ControlMode::Velocity, 0.0);
			rightMasterMotor->Set(ControlMode::Velocity, 0.0);
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
			//TODO npos CORRESPONDS TO "NOTHING FOUND", FLIP LOGIC?
			//PARSE CODE FOR SWITCH/SCALE POSITION FINDING
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
			//TODO WRITE DEBUG STATEMENTS
		}

		switch (switchposition) {
		//switchposition is the variable determining switch position, left or right
		case LEFT_FIELD:
			switch (fieldPos) {
			//fieldPos is the variable determining robot position on the field , left, center, middle
			case ROB_ON_LEFT:
				//SWITCH AND CASE VALUES HAVE NO MEANING (YET)
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
		allianceColor = to_string(driverStation->GetInstance().GetAlliance());
		SmartDashboard::PutString("Alliance Color", allianceColor);
		networkTableData->PutString("Alliance Color", allianceColor);
		SEGMENT_SELECTION(DriverStation::GetInstance().GetGameSpecificMessage());
		SHIFT_LOW();
		isHighGear = false;
		SHIFT_HIGH();
		driveState = SEG_1;

		autonomousTimer->Reset();
		autonomousTimer->Start();
		NAVXBoard->Reset();
		ADXGyro->Reset();
	}

	void AutonomousPeriodic() {
		autonomousRange = roborioUltrasonic->GetAverageValue();
		//SELECTION BASED ON SWITCH POSTION, OR ROBOT position
		//isTracking WILL NEED TO BE RETRIEVED FROM NETWORKTABLES
		//TIME WILL NEED TO BE SENT TO JEFF ALONG WITH CURRENT DISTANCE
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
	}

	void TeleopPeriodic() {
		//THRESHHOLDS SHOULD KEEP THE JOYSTICK FROM MOVING ROBOT AS LONG AS INPUT IS < 5%
		if(rightJoystick->GetY() < 0.05) {
			throttle = 0;
		} else {
			throttle = rightJoystick->GetY();
		}
		if(rightJoystick->GetX() < 0.05) {
			steer = 0;
		} else {
			steer = rightJoystick->GetX();
		}

		leftMasterMotor->Set(ControlMode::PercentOutput, throttle - steer);
		rightMasterMotor->Set(ControlMode::PercentOutput, -throttle - steer);
		if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && soloTest) {
			//soloTest KEEPS THE IF LOOP FROM RUNNING CONTINUOSLY
			//IF BUTTON IS PRESSED, AND SINCE soloTest IS TRUE, IF LOOP WILL RUN, ONCE
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

		leftVelocity = leftMasterMotor->GetSelectedSensorVelocity(1); //*SOME SCALE FACTOR
		leftVelocity = rightMasterMotor->GetSelectedSensorVelocity(3);

		roborioUltrasonic->GetAverageValue();

		SmartDashboard::PutNumber("RPMS_L", leftVelocity);
		SmartDashboard::PutNumber("RPMS_R", leftVelocity);
		SmartDashboard::PutNumber("ADX_OUTPUT", ADXGyro->GetAngle());
		SmartDashboard::PutNumber("NAVX_OUTPUT", NAVXBoard->GetAngle());
		SmartDashboard::PutNumber("COMPOSITE_OUTPUT", combinedGyroValue);
		SmartDashboard::PutNumber("RANGE", roborioUltrasonic->GetVoltage());

		//MEASURED IN NATIVE UNITS PER 100MS
		//MUST CONVERT TO REAL UNITS
		SmartDashboard::PutNumber("Left Encoder Velocity", (gearRatio)*(leftMasterMotor->GetSelectedSensorVelocity(0)));
		SmartDashboard::PutNumber("Right Encoder Velocity", (gearRatio)*(rightMasterMotor->GetSelectedSensorVelocity(0)));
		SmartDashboard::PutNumber("Left Encoder Position", (gearRatio)*(leftMasterMotor->GetSelectedSensorPosition(0)));
		SmartDashboard::PutNumber("Right Encoder Position", (gearRatio)*(rightMasterMotor->GetSelectedSensorPosition(0)));
		frc::Wait(0.005);
	}

	void TestPeriodic() {

	}

private:

};

START_ROBOT_CLASS(Robot)
