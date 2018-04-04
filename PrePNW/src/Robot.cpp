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
#include "Ultrasonic.h"
//Utilities header files.
#include <DriverStation.h>
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

	struct Leg_Scale_Data{
		double SCALE_DIST;
		double SCALE_ANGLE;
	}scaleSegments[4];

	//Creates enum values based on robot positioning for use in autonomous.
	enum robotStartingPosition{
		BOT_ON_LEFT = 0,
		BOT_ON_CENTER = 1,
		BOT_ON_RIGHT = 2,
		BOT_POS_FAILURE = 3,
	};

	//Enums for deciding between left/right switch position.
	enum autonomousSwitchData{
		SWITCH_LEFT = 1,
		SWITCH_RIGHT = 2,
		SWTICH_FAILURE = -1,
	};

	//Enums for deciding between left/right scale position.
	enum autonomousScaleData{
		SCALE_LEFT = 1,
		SCALE_RIGHT = 2,
		SCALE_FAILURE = -1,
	};

	//Enum values for the joysticks for intuitiveness.
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
	//Overide for the soft limits.
	bool isOverride;
	//Booleans for toggle state testing.
	bool shiftingSoloTest;
	bool actuatorSoloTest;
	bool controlSoloTest;
	bool overideSoloTest;
	//Master toggle for swapping control schemes.
	bool teleopMasterSwitch;

	bool doneDriving;
	bool doneTurning;
	//TODO Localize this variable to the turning function?
	bool targetReached;
	//Int to decide whether to be in swtich or scale mode.
	//Also coordinates whether to be in high or low gear.
	int switchScale;
	//Limiter for autonomous to finish running various segments.
	//At the end of the autonomous code.
	int maxSegmentCount;
	//Control variable used for scheduling which "segment" to carry out.
	//Whether it be turning, or driving forward.
	int driveState;
	//Value for keeping track of which gear ratio to be using.
	double gearRatio;
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
	const float valveWait = 0.25;
	//Gear ratios for low/high gears.
	//Previously 2.0/15.0
	//TODO Must decide which is the most accurate.
	const float standardGearRatio = (1.0/15.0);
	const float lowGearRatio = (1.0/5.0);
	const float highGearRatio = (6.0/17.0);
	//Two throttle values for autonomous.
	const double lowGearMotorVelocity = 0.5;
	const double highGearMotorVelocity = 0.5;
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
		isOverride = false;

		shiftingSoloTest = true;
		actuatorSoloTest = true;
		controlSoloTest = true;
		overideSoloTest = true;

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
		intakeMasterMotor->ConfigPeakOutputForward(0.25, 10);
		intakeMasterMotor->ConfigPeakOutputReverse(-0.75, 10);

		intakeSlaveMotor->ConfigPeakOutputForward(0.65, 10);
		intakeSlaveMotor->ConfigPeakOutputReverse(-0.25, 10);

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
	void SHIFT_HIGH () {
		gearBox->Set(DoubleSolenoid::kForward);
		Wait(0.5);
		gearBox->Set(DoubleSolenoid::kOff);
		isHighGear =! isHighGear;
		//Where green is high gear, and red is low gear.
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
	void INTAKE_ARM_IN() {
		intakeArmActuator->Set(DoubleSolenoid::kForward);
		Wait(0.5);
		intakeArmActuator->Set(DoubleSolenoid::kOff);
		isActuator =! isActuator;
		//Where green is in, and red is out.
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
	void Baseline() {
		if(autonomousTimer->Get() < 6.0) {
			leftMasterMotor->Set(ControlMode::PercentOutput, -0.5);
			rightMasterMotor->Set(ControlMode::PercentOutput, -0.5);
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		}
	}
	void AutonomousInit() override {
		SHIFT_HIGH();
//		DATA_SET(DriverStation::GetInstance().GetGameSpecificMessage());

		//O = switch, 1 = scale.
//		switchScale = SmartDashboard::GetNumber("Scale?", 0);
//		if(switchScale == 0) {
//			SHIFT_LOW();
//			gearRatio = lowGearRatio;
//		} else if(switchScale == 1) {
//			SHIFT_HIGH();
//			gearRatio = highGearRatio;
//		}

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
		if(autonomousTimer->Get() < 15.0) {
			Baseline();
		}
/*
		//This block should be able to drive to the switch.
		//Uses a logic system that resets and moves forward one tick.
		if(autonomousTimer->Get() < 2.0) {
			actuatorMotor->Set(ControlMode::PercentOutput, 0.125);
		} else if(autonomousTimer->Get() < 15.0) {
			elevatorMasterMotor->Set(ControlMode::Position, 10000);
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
*/
	}

	void TeleopInit() {
		//Shuts off timer to save on memory, resets navigation sensors.
		autonomousTimer->Stop();
		NAVXBoard->Reset();
		ADXGyro->Reset();

		//Sets all active motors to zero percent output for safety.
		leftMasterMotor->Set(ControlMode::PercentOutput, 0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0);
		elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);

		//This function should always put us into low gear, regardless of previous gear setting.
		if(!isHighGear){
			SHIFT_LOW();
		}
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

		//This block will activate the rumble function in the XBox controls if the elevator spool is above/below a position.
		if((elevatorMasterMotor->GetSelectedSensorPosition(0) > 0) && (elevatorMasterMotor->GetSelectedSensorPosition(0) < 1000)) {
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
		} else if((elevatorMasterMotor->GetSelectedSensorPosition(0) > 45000) && (elevatorMasterMotor->GetSelectedSensorPosition(0) < 48000)) {
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.25);
		} else {
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.0);
			XboxGamepad->SetRumble(GenericHID::kLeftRumble, 0.0);
		}
		SmartDashboard::PutNumber("Elevator Position", elevatorMasterMotor->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Elevator Master", elevatorMasterMotor->GetMotorOutputPercent());
		SmartDashboard::PutNumber("Elevator Slave", elevatorMasterMotor->GetMotorOutputPercent());
	}

	void TestPeriodic() {

	}
private:
};
START_ROBOT_CLASS(Robot)
