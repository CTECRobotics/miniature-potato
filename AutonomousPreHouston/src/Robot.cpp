#include <iostream>
#include <string>
#include <memory>
#include <stdlib.h>
//Main header files.
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <IterativeRobot.h>
#include "WPILib.h"
//Peripheral header files.
#include "Doublesolenoid.h"
#include "Timer.h"
#include <AnalogGyro.h>
#include <cmath>
#include <ADXRS450_Gyro.h>
//Utilities header files.
//Various namespaces for efficiency.
using namespace std;
using namespace nt;

class Robot : public frc::IterativeRobot {
public:
	//Creates three Leg_Data structs as writable segments for use in autonomous control.
	struct Leg_Switch_Data{
		double SWITCH_DIST;
		double SWITCH_ANGLE;
	}switchSegments[3];

	enum rightJoystickMap{
		ACTUATOR_TOGGLE = 1,
		GEAR_CHANGE = 2,
	};

	//Enum values for the XBox for intuitiveness.
	enum XboxGamepadMap{
		JOYSTICK_ACTUATOR_UP = 180,
		JOYSTICK_ACTUATOR_DOWN = 0,
		ELEVATOR_AXIS = 1,
		INTAKE_AXIS = 5,
		GAMEPAD_ACTUATOR_TOGGLE = 1,
	};
	//State checkers to ensure various proper operation for toggling.
	bool isHighGear;
	bool isInHighGear;
	//State checkers to ensure proper toggling operation for pneumatic actuating.
	bool isActuator;
	bool isArmIn;
	//Booleans for toggle state testing.
	bool shiftingSoloTest;
	bool actuatorSoloTest;

	bool targetReached;

	bool doneDriving;
	bool doneTurning;

	bool overrideShift;
	bool overrideAutonomous;

	int robotPos;

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
	//Variable for use in the TURN_TO_ANGLE function as the setpoint.
	double autonomousAngleSet;
	//Variable for use in the DRIVE_TO_DISTANCE function as the setpoint.
	double autonomousDistanceSet;
	//Wait time to allow air to flow through the system and shift the gears.
	const float valveWait = 0.125;

	const float gearRatio = 1.0/15.0;

	const double motorVelocity = 0.5;

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
	//Joystick and XBOX api is bundled in with the joystick libraries.
	Joystick *leftJoystick;
	Joystick *rightJoystick;
	Joystick *XboxGamepad;
	void RobotInit() {
		isHighGear = false;
		isActuator = false;

		shiftingSoloTest = true;
		actuatorSoloTest = true;

		targetReached = false;

		doneDriving = false;
		doneTurning = false;

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

		intakeMasterMotor = new TalonSRX(6);
		intakeSlaveMotor = new TalonSRX(8);

		actuatorMotor = new TalonSRX(9);

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

		intakeMasterMotor->Set(ControlMode::PercentOutput, 0);
		intakeSlaveMotor->Set(ControlMode::PercentOutput, 0);

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

		intakeSlaveMotor->SetInverted(true);
		actuatorMotor->SetInverted(true);

		//Actuator drive motor limited downward.
		actuatorMotor->ConfigPeakOutputForward(0.5, 10);
		actuatorMotor->ConfigPeakOutputReverse(-1.0, 10);
		//IntakeMotors drive offset purposefully
		intakeMasterMotor->ConfigPeakOutputForward(0.30, 10);
		intakeMasterMotor->ConfigPeakOutputReverse(-0.80, 10);

		intakeSlaveMotor->ConfigPeakOutputForward(0.70, 10);
		intakeSlaveMotor->ConfigPeakOutputReverse(-0.30, 10);

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
	bool TURN_TO_ANGLE (int autonomousAngleSet) {

		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);

		double tolerance = 9.0;

		//All positive motor values will rotate robot left.
		//Inversely, all negative will rotate robot right.
		if((!targetReached && (combinedGyroValue < autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
			rightMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
			doneTurning = false;
			if((combinedGyroValue > autonomousAngleSet - tolerance) && (
					combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			SmartDashboard::PutString("Motion State", "Turning Left!");

		} else if(!targetReached && ((combinedGyroValue > autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
			rightMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
			doneTurning = false;
			if((combinedGyroValue > autonomousAngleSet - tolerance) &&
					(combinedGyroValue < autonomousAngleSet + tolerance)) {
				targetReached = true;
			}
			SmartDashboard::PutString("Motion State", "Turning Right!");

		}
		if(targetReached){
			leftMasterMotor->Set(ControlMode::PercentOutput, (0));
			rightMasterMotor->Set(ControlMode::PercentOutput, (0));
			doneTurning = true;
			SmartDashboard::PutString("Motion State", "Done Turning!");
			targetReached = true;
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

		if((((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick) &&
				(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick)))) {

			leftMasterMotor->Set(ControlMode::PercentOutput, motorVelocity);
			rightMasterMotor->Set(ControlMode::PercentOutput, motorVelocity);

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

		//0 = left, 1 = right
		if(inputData == "LLL") {
			switchPos = 0;
		} else if(inputData == "RRR") {
			switchPos = 1;
		} else if(inputData == "LRL") {
			switchPos = 0;
		} else if(inputData == "RLR") {
			switchPos = 1;
		} else {
			switchPos = -1;
		}

		if(switchPos == robotPos) {
			//All angle values are headings.
			//Negative headings turn left, postive turn right.

			switch(switchPos) {
			case 1:
				switchSegments[0].SWITCH_DIST = 167.65;
				switchSegments[0].SWITCH_ANGLE = 90.0;
				switchSegments[1].SWITCH_DIST = 40.0;
				switchSegments[1].SWITCH_ANGLE = 90.0;
				switchSegments[2].SWITCH_DIST = 0.0;
				switchSegments[2].SWITCH_ANGLE = 90.0;
				break;
			case 0:
				switchSegments[0].SWITCH_DIST = 167.65;
				switchSegments[0].SWITCH_ANGLE = -90.0;
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
		} else {

		}

	}
	void SHIFT_HIGH () {
		gearBox->Set(DoubleSolenoid::kForward);
		Wait(valveWait);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
		//Where green is high gear, and red is low gear.
		isInHighGear = false;
		SmartDashboard::PutBoolean("Gear State", isInHighGear);
	}
	void SHIFT_LOW () {
		gearBox->Set(DoubleSolenoid::kReverse);
		Wait(valveWait);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
		isInHighGear = true;
		SmartDashboard::PutBoolean("Gear State", isInHighGear);
	}
	void INTAKE_ARM_IN() {
		intakeArmActuator->Set(DoubleSolenoid::kForward);
		Wait(valveWait);
		intakeArmActuator->Set(DoubleSolenoid::kOff);
		isActuator =! isActuator;
		//Where green is in, and red is out.
		isArmIn = false;
		SmartDashboard::PutBoolean("Actuator State", isArmIn);
	}
	void INTAKE_ARM_OUT() {
		intakeArmActuator->Set(DoubleSolenoid::kReverse);
		Wait(valveWait);
		intakeArmActuator->Set(DoubleSolenoid::kOff);
		isActuator =! isActuator;
		isArmIn = true;
		SmartDashboard::PutBoolean("Actuator State", isArmIn);
	}
	void Baseline() {
		if(autonomousTimer->Get() < 4.75) {
			leftMasterMotor->Set(ControlMode::PercentOutput, -0.5);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.5);
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);
		}
	}
	void AutonomousInit() override {
		SHIFT_HIGH();

		//0 = left, 1 = right
		robotPos = SmartDashboard::GetNumber("Robot Position", -1);
		//false = no override, true = override.
		overrideShift = SmartDashboard::GetBoolean("Shifting Override", false);
		//false = no override, true = override.
		overrideAutonomous = SmartDashboard::GetBoolean("Autonomous Override", false);

		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);

		autonomousTimer->Reset();
		autonomousTimer->Start();
		NAVXBoard->Reset();
		ADXGyro->Reset();
	}
	void AutonomousPeriodic() {
		if(autonomousTimer->Get() < 15.0) {
			Baseline();
			SmartDashboard::PutNumber("Gyro Value", ADXGyro->GetAngle());
		}
	}
	void TeleopInit() {
		//Shuts off timer to save on memory, resets navigation sensors.
		autonomousTimer->Stop();

		//Sets all active motors to zero percent output for safety.
		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);

		SHIFT_LOW();
	}
	void TeleopPeriodic() {
		//This block coordinates toggling the gear mode, high or low.
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
		//Following two blocks control robot's throttle and steering.
		//Integrates 5% thresholds for the throttle and steering.
		if(abs(leftJoystick->GetY()) < 0.05) {
			robotThrottle = 0;
		} else {
			robotThrottle = leftJoystick->GetY();
		}

		if(abs(rightJoystick->GetX()) < 0.05) {
			robotSteer = 0;
		} else {
			robotSteer = rightJoystick->GetX();
		}
		leftMasterMotor->Set(ControlMode::PercentOutput, robotThrottle - robotSteer);
		rightMasterMotor->Set(ControlMode::PercentOutput, -robotThrottle - robotSteer);

		//This block coordinates toggling the in/out state of the arms.
		//Reuses the logic from the shifting block.
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

		//This block controls the upward/downward motion of the actuator.
		if(XboxGamepad->GetPOV() == XboxGamepadMap::JOYSTICK_ACTUATOR_DOWN) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.5);
		} else if(XboxGamepad->GetPOV() == XboxGamepadMap::JOYSTICK_ACTUATOR_UP) {
			actuatorMotor->Set(ControlMode::PercentOutput, -0.5);
		} else {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.0);
		}

		//This block controls elevator lift throttle.
		//Integrates 5% thresholds for the elevator lift throttle.
		if(abs(XboxGamepad->GetRawAxis(XboxGamepadMap::ELEVATOR_AXIS)) < 0.05) {
			elevatorThrottle = 0;
		} else {
			elevatorThrottle = XboxGamepad->GetRawAxis(XboxGamepadMap::ELEVATOR_AXIS);
		}

		elevatorMasterMotor->Set(ControlMode::PercentOutput, -elevatorThrottle);

		//This block controls intake throttle.
		//Integrates 5% thresholds for the intake throttle.
		if(abs(XboxGamepad->GetRawAxis(XboxGamepadMap::INTAKE_AXIS)) < 0.05) {
			intakeThrottle = 0;
		} else {
			intakeThrottle = XboxGamepad->GetRawAxis(XboxGamepadMap::INTAKE_AXIS);
		}

		intakeMasterMotor->Set(ControlMode::PercentOutput, -intakeThrottle);
		intakeSlaveMotor->Set(ControlMode::PercentOutput, -intakeThrottle);

		SmartDashboard::PutNumber("Elevator Power", elevatorMasterMotor->GetMotorOutputPercent());

		SmartDashboard::PutNumber("Gyro Value", ADXGyro->GetAngle());
	}

	void TestPeriodic() {

	}
private:
};
START_ROBOT_CLASS(Robot)
