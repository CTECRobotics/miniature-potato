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
	bool isHighGear;
	bool soloTest;
	double throttle;
	double steer;
	const float encoderRotTick = 4096.0;		//4096 "TICKS" PER ROTATION ACCORDING TO MAGNETIC ENCODER SPECS
	const float PI = 3.1415;
	const float circumference = (PI*6.0);
	const float ultrasonicConversion = 5000/4.88;	//5000 MM PER 4.88 VOLTAGE ACCORDING TO ULTRASONIC SPECS
	const float inchToMeter = (1.0/39.4);
	const float gearRatio = (2.0/15.0);
	double navxGyro;
	double rioGyro;
	double combinedGyroValue;
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
	void RobotInit() {
		isHighGear = false;
		soloTest = true;
		throttle = 0;
		steer = 0;
		navxGyro = 0;
		rioGyro = 0;
		combinedGyroValue = 0;
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
		leftMasterMotor->GetSelectedSensorVelocity(0);
		leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		leftMasterMotor->SetSensorPhase(true);
		leftMasterMotor->ConfigNominalOutputForward(0, 10);
		leftMasterMotor->ConfigNominalOutputReverse(0, 10);
		leftMasterMotor->ConfigPeakOutputForward(1, 10);
		leftMasterMotor->ConfigPeakOutputReverse(-1, 10);
		leftMasterMotor->Config_kF(0, 0.0, 10);
		leftMasterMotor->Config_kP(0, 0.029, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.0, 10);

		rightMasterMotor->GetSelectedSensorVelocity(0);
		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSensorPhase(false);
		rightMasterMotor->ConfigNominalOutputForward(0, 10);
		rightMasterMotor->ConfigNominalOutputReverse(0, 10);
		rightMasterMotor->ConfigPeakOutputForward(1, 10);
		rightMasterMotor->ConfigPeakOutputReverse(-1, 10);
		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.029, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.0, 10);

		autonomousTimer->Reset();

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
	void TURN_TO_ANGLE (double autonomousAngleSet) {
		const float inchToMeter = (1.0/39.4);
		const float drivetrainRadius = 12.5625;
		const float angletolerance = 2.0;
		float difference = 0.0;
		float distancePerWheel = 0;
		double direction;
		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);

		//Where a positive difference is left, negative being right.
		difference = (((-combinedGyroValue + autonomousAngleSet)) * (PI/180));
		//Angular difference in radians for use in determining direction.
		distancePerWheel = (drivetrainRadius * inchToMeter * abs(difference));
		//Distance needed to be traveled by each side of the robot.
		//Essentially S = r * (theta).
		direction = ((sin(difference))/abs(sin(difference)));

		if((combinedGyroValue > (autonomousAngleSet - angletolerance)) &&
				(combinedGyroValue < (autonomousAngleSet + angletolerance))) {
			leftMasterMotor->Set(ControlMode::Position, -(distancePerWheel/(circumference * inchToMeter)) * encoderRotTick * direction);
			rightMasterMotor->Set(ControlMode::Position, (distancePerWheel/(circumference * inchToMeter)) * encoderRotTick * direction);
			SmartDashboard::PutString("Turning State", "Turning!");
			SmartDashboard::PutNumber("Angle Set Point", autonomousAngleSet);
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			SmartDashboard::PutString("Turning State", "Done!");
		}
		SmartDashboard::PutNumber("Distance Per Wheel, Meters", distancePerWheel);
		SmartDashboard::PutNumber("Distance Per Wheel, Units", ((distancePerWheel/(circumference * inchToMeter)) * encoderRotTick * direction));
		SmartDashboard::PutNumber("Difference", difference);
		SmartDashboard::PutNumber("Direction", direction);
		SmartDashboard::PutNumber("Current Angle", combinedGyroValue);
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
	}

	void AutonomousPeriodic() {

		if(autonomousTimer->Get() < 15.0) {
			TURN_TO_ANGLE(-90.0);
			SmartDashboard::PutNumber("Time", autonomousTimer->Get());
			SmartDashboard::PutNumber("LM Output", leftMasterMotor->GetMotorOutputPercent());
			SmartDashboard::PutNumber("RM Output", rightMasterMotor->GetMotorOutputPercent());
			SmartDashboard::PutNumber("LS Output", leftSlaveMotor->GetMotorOutputPercent());
			SmartDashboard::PutNumber("RS Output", rightSlaveMotor->GetMotorOutputPercent());
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
		NAVXBoard->Reset();
		ADXGyro->Reset();

	}

	void TeleopPeriodic() {
		if(abs(rightJoystick->GetY()) < 0.05) {
			throttle = 0;
		} else {
			throttle = rightJoystick->GetY();
		}
		if(abs(rightJoystick->GetX()) < 0.05) {
			steer = 0;
		} else {
			steer = rightJoystick->GetX();
		}
		leftMasterMotor->Set(ControlMode::PercentOutput, throttle - steer);
		rightMasterMotor->Set(ControlMode::PercentOutput, -throttle - steer);

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

		SmartDashboard::PutNumber("ADX_OUTPUT", ADXGyro->GetAngle());
		SmartDashboard::PutNumber("NAVX_OUTPUT", NAVXBoard->GetAngle());
		SmartDashboard::PutNumber("COMPOSITE_OUTPUT", combinedGyroValue);

		frc::Wait(0.005);
	}

	void TestPeriodic() {

	}
private:
};

START_ROBOT_CLASS(Robot)
