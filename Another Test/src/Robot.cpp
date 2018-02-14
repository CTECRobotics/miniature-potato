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
		leftMasterMotor->Config_kP(0, 0.0075, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.50, 10);
		//Sets up the acceleration and cruise velocity for the MotionMagic Control.
		leftMasterMotor->ConfigMotionCruiseVelocity(2048, 10);
		leftMasterMotor->ConfigMotionAcceleration(1024, 10);

		rightMasterMotor->GetSelectedSensorVelocity(0);
		rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
		rightMasterMotor->SetSensorPhase(false);
		rightMasterMotor->ConfigNominalOutputForward(0, 10);
		rightMasterMotor->ConfigNominalOutputReverse(0, 10);
		rightMasterMotor->ConfigPeakOutputForward(1, 10);
		rightMasterMotor->ConfigPeakOutputReverse(-1, 10);
		rightMasterMotor->Config_kF(0, 0.0, 10);
		rightMasterMotor->Config_kP(0, 0.0075, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.50, 10);
		rightMasterMotor->ConfigMotionCruiseVelocity(2048, 10);
		rightMasterMotor->ConfigMotionAcceleration(1024, 10);

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
	void DRIVE_TO_DISTANCE(int autonomousDistanceSet) {
		rightMasterMotor->SetInverted(true);
		rightSlaveMotor->SetInverted(true);
		rightMasterMotor->SetSensorPhase(true);

		const float inchToMeter = (1.0/39.4);
		if((((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 0.15)/(circumference * inchToMeter)) * encoderRotTick) &&
				(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 0.15)/(circumference * inchToMeter)) * encoderRotTick)))) {
			leftMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);
			rightMasterMotor->Set(ControlMode::Position, -(autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);

			SmartDashboard::PutNumber("Left Encoder Position", (gearRatio)*(leftMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Right Encoder Position", (gearRatio)*(rightMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Distance to Drive", (autonomousDistanceSet/(circumference * inchToMeter)) * encoderRotTick);
			SmartDashboard::PutNumber("Distance in Meters", autonomousDistanceSet);
			SmartDashboard::PutString("Driving State?", "Currently Moving Forward");

		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);

			SmartDashboard::PutNumber("Left Encoder Position", (gearRatio)*(leftMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutNumber("Right Encoder Position", (gearRatio)*(rightMasterMotor->GetSelectedSensorPosition(0)));
			SmartDashboard::PutString("Driving State?", "Done!");

		}
	}
	void TURN_TO_ANGLE (int autonomousAngleSet) {
		//Distance from the geometric center of the robot to the center contact points of the wheels.
		//Due to West Coast drive.
		const float drivetrainRadius = 12.5625;
		float difference = 0.0;
		double direction;
		float distancePerWheel = 0;
		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);
		leftMasterMotor->Config_kP(0, 0.029, 10);
		leftMasterMotor->Config_kI(0, 0.0, 10);
		leftMasterMotor->Config_kD(0, 0.00015, 10);
		leftMasterMotor->Config_kF(0, 0.125, 10);
		rightMasterMotor->Config_kP(0, 0.029, 10);
		rightMasterMotor->Config_kI(0, 0.0, 10);
		rightMasterMotor->Config_kD(0, 0.00015, 10);
		rightMasterMotor->Config_kF(0, 0.125, 10);

		difference = (((autonomousAngleSet - combinedGyroValue)) * (PI/180));
		//Angular difference in radians for use in determining direction.
		distancePerWheel = abs(drivetrainRadius * difference);
		//Distance needed to be traveled by each side of the robot.
		//Essentially S = r * (theta).
		direction = ((sin(difference)))/abs(sin(difference));
		if(direction == 1){
			rightMasterMotor->SetInverted(true);
			rightSlaveMotor->SetInverted(true);
			rightMasterMotor->SetSensorPhase(true);
			leftMasterMotor->SetInverted(true);
			leftSlaveMotor->SetInverted(true);
			leftMasterMotor->SetSensorPhase(true);
		}
		if(direction == -1){
			rightMasterMotor->SetInverted(false);
			rightSlaveMotor->SetInverted(false);
			rightMasterMotor->SetSensorPhase(false);
			leftMasterMotor->SetInverted(false);
			leftSlaveMotor->SetInverted(false);
			leftMasterMotor->SetSensorPhase(false);
		}
		if(((direction == 1) && (combinedGyroValue < autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::MotionMagic, (direction * (distancePerWheel/(circumference)) * encoderRotTick));
			rightMasterMotor->Set(ControlMode::MotionMagic, -(direction * -(distancePerWheel/(circumference)) * encoderRotTick));
		} else if(((dirrection == -1) && (combinedGyroValue < autonomousAngleSet))) {
			leftMasterMotor->Set(ControlMode::MotionMagic, -(direction * (distancePerWheel/(circumference)) * encoderRotTick));
			rightMasterMotor->Set(ControlMode::MotionMagic, -(direction * -(distancePerWheel/(circumference)) * encoderRotTick));
		} else {
			leftMasterMotor->Set(ControlMode::PercentOutput, 0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 0);
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


		autonomousTimer->Reset();
		autonomousTimer->Start();
		NAVXBoard->Reset();
		ADXGyro->Reset();
	}

	void AutonomousPeriodic() {

		if(autonomousTimer->Get() < 15.0) {
			TURN_TO_ANGLE(90.0);
//			DRIVE_TO_DISTANCE(2)	;
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
		leftMasterMotor->SetSensorPhase(true);
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

		//TODO Create ludicrous mode, but don't tell J03.
		navxGyro = NAVXBoard->GetAngle();
		rioGyro = ADXGyro->GetAngle();
		combinedGyroValue = ((navxGyro + rioGyro)/2);
		SmartDashboard::PutNumber("Left Position", leftMasterMotor->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Right Position", rightMasterMotor->GetSelectedSensorPosition(0));
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
