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

class Robot : public frc::IterativeRobot {
public:
		//REGULATORY VALUES
	double VALVE_L;
	double VALVE_R;
	int AUTO_MODE;
		//CONTROL VALUES
	double CAMERA_ERROR;	//POSTIVE IS NEGATIVE, RIGHT IS POSITIVE
	double GYRO_SETPOINT;
	double CURRENT_SETPOINT;
	double DISTANCE;
	double RANGE_METERS;
	double OUTPUT;
	double THROTTLE;
	double STEER;
	std::string ALLIANCE;
		//DON'T TOUCH ME VALUES
	double MAXIMUM_RPM;
	double VALVE_WAIT;
		//MISC


		//CANTALON OBJECTS
	TalonSRX *MOTOR_LM;
	TalonSRX *MOTOR_LS;
	TalonSRX *MOTOR_RM;
	TalonSRX *MOTOR_RS;
		//PID ASSESTS
		//PID LOOPS
	PIDController *MOTOR_L_PID;
	PIDController *MOTOR_R_PID;
		//SOLENOIDS
	Solenoid *LAUNCHER_L;
	Solenoid *LAUNCHER_R;
	DoubleSolenoid *GEARBOX_MAIN;
		//SENSOR INPUTS
	Encoder *MOTOR_ENCODER_L;
	Encoder *MOTOR_ENCODER_R;
	Encoder *SPINNER_RPM;
	ADXRS450_Gyro *CONTROL_GYRO;
	Potentiometer *TURRET_CONTROLLER;
		//MISC
	AHRS *PHYSIX;
	Timer *AUTO_TIMER;
	Joystick *LEFT_JOYSTICK;
	Joystick *RIGHT_JOYSTICK;
	NetworkTable *VISION_DATA;
	AnalogInput *AUTO_RANGEFINDER;
	Accelerometer *INTERNAL_ACCELEROMETER;
	DriverStation *DS;
	void RobotInit() {
				//VALUE SETUP
			//REGULATORY VALUES
		VALVE_L = 0;
		VALVE_R = 1;
		AUTO_MODE = 0;
			//CONTROL VALUES
		CAMERA_ERROR = 0;
		GYRO_SETPOINT = 0;
		CURRENT_SETPOINT = 0;
		DISTANCE = 0;
		OUTPUT = 0;
		THROTTLE = 0;
		STEER = 0;
		ALLIANCE = "";
			//DO NOT MODIFY
		MAXIMUM_RPM = 511;
		VALVE_WAIT = 0.25;
				//PERIPHERAL SETUP
			//TALONSRX
		MOTOR_LM = new TalonSRX(1);
		MOTOR_LS = new TalonSRX(2);
		MOTOR_RM = new TalonSRX(3);
		MOTOR_RS = new TalonSRX(4);
			//ENCODERS
		MOTOR_ENCODER_L = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
		MOTOR_ENCODER_L->SetMaxPeriod(.1);
		MOTOR_ENCODER_L->SetMinRate(10);
		MOTOR_ENCODER_L->SetDistancePerPulse(5);
		MOTOR_ENCODER_L->SetReverseDirection(false);
		MOTOR_ENCODER_L->SetSamplesToAverage(7);

		MOTOR_ENCODER_R = new Encoder(2, 3, false, Encoder::EncodingType::k4X);
		MOTOR_ENCODER_R->SetMaxPeriod(.1);
		MOTOR_ENCODER_R->SetMinRate(10);
		MOTOR_ENCODER_R->SetDistancePerPulse(5);
		MOTOR_ENCODER_R->SetReverseDirection(false);
		MOTOR_ENCODER_R->SetSamplesToAverage(7);
			//NAVX CONNECTION ATTEMPT
        try {
            PHYSIX = new AHRS(SPI::Port::kMXP);
        } catch (std::exception ex ) {
            std::string ERROR_STRING = "Error instantiating navX-MXP: PHYSIX";
            ERROR_STRING += ex.what();
            DriverStation::ReportError(ERROR_STRING.c_str());
        }
        	//SENSORS, JOYSTICKS, ACCELEROMETERS, ETC.
		LEFT_JOYSTICK = new Joystick(0);
		CONTROL_GYRO = new ADXRS450_Gyro();
		AUTO_RANGEFINDER = new AnalogInput(0);
		AUTO_TIMER = new Timer();
		INTERNAL_ACCELEROMETER = new BuiltInAccelerometer();
			//INITIAL SETPOINTS, CALIB, ETC.
		MOTOR_LM->Set(ControlMode::PercentOutput, 0);
		MOTOR_RM->Set(ControlMode::PercentOutput, 0);
		CONTROL_GYRO->Calibrate();
		MOTOR_LS->Set(ControlMode::Follower, 1);
		MOTOR_RS->Set(ControlMode::Follower, 3);
			//MISC

		//MOTOR_L_PID = new PIDController(1, 1, 1, MOTOR_ENCODER_L, MOTOR_LM);
		//MOTOR_R_PID = new PIDController(1, 1, 1, MOTOR_ENCODER_R, MOTOR_RM);
		VISION_DATA = NetworkTable::GetTable("JETSON");

	}
	void AutonomousInit() override {
		ALLIANCE = std::to_string(DS->GetInstance().GetAlliance());
		SmartDashboard::PutString("ALLIANCE_COLOR", "ALLIANCE");


		//MOTOR_ENCODER_L->Reset();
		//MOTOR_ENCODER_R->Reset();

		//MOTOR_L_PID->Disable();
		//MOTOR_R_PID->Disable();

		AUTO_TIMER->Reset();
		AUTO_TIMER->Start();

		GYRO_SETPOINT = CONTROL_GYRO->GetAngle();


		//CONTROL_GYRO->Reset();

		//MOTOR_L_PID->SetSetpoint(MAXIMUM_RPM * 0.8);
		//MOTOR_R_PID->SetSetpoint(MAXIMUM_RPM * 0.8);
	}

	void AutonomousPeriodic() {
		AUTO_MODE = VISION_DATA->GetNumber("AUTO_MODE", 0);
		switch (AUTO_MODE) {
		case 0:
			while(true) {

			}
			break;
		case 1:
			while(true) {

			}
			break;
		case 2:
			while(true) {

			}
			break;
		}
		/*
		if(AUTO_TIMER->Get() >= 0) {
			if(AUTO_RANGEFINDER->GetValue() >= 1000) {
				MOTOR_LM->Set(ControlMode::PercentOutput, 0.25);
				MOTOR_RM->Set(ControlMode::PercentOutput, -0.28);
			} else if (AUTO_RANGEFINDER->GetValue() < 1000) {
				MOTOR_LM->Set(ControlMode::PercentOutput, 0.0);
				MOTOR_RM->Set(ControlMode::PercentOutput, 0.0);
				//DO SOMETHING
			} else {
				MOTOR_LM->Set(ControlMode::PercentOutput, 0.0);
				MOTOR_RM->Set(ControlMode::PercentOutput, 0.0);
			}
		}

		if(AUTO_TIMER->Get() >= 14) {
			MOTOR_LM->Set(ControlMode::PercentOutput, 0.25);
			MOTOR_RM->Set(ControlMode::PercentOutput, -0.28);
		} else if(AUTO_TIMER->Get() >= 1) {
			if(CONTROL_GYRO->GetAngle() <= (GYRO_SETPOINT-2)) {
				MOTOR_LM->Set(ControlMode::PercentOutput, 0.15);
				MOTOR_RM->Set(ControlMode::PercentOutput, -0.08);
			} else if(CONTROL_GYRO->GetAngle() >= (GYRO_SETPOINT+2)) {
				MOTOR_LM->Set(ControlMode::PercentOutput, 0.05);
				MOTOR_RM->Set(ControlMode::PercentOutput, -0.18);
			} else if(CONTROL_GYRO->GetAngle() > (GYRO_SETPOINT-2) || CONTROL_GYRO->GetAngle() < (GYRO_SETPOINT+2)) {
				MOTOR_LM->Set(ControlMode::PercentOutput, 0.25);
				MOTOR_RM->Set(ControlMode::PercentOutput, -0.28);
			}
		} else if(AUTO_TIMER->Get() >= 0) {
			MOTOR_LM->Set(0);
			MOTOR_RM->Set(0);
		}
		*/
	}

	void TeleopInit() {
		AUTO_TIMER->Stop();
		//MOTOR_L_PID->Disable();
		//MOTOR_R_PID->Disable();
	}

	void TeleopPeriodic() {
		THROTTLE = LEFT_JOYSTICK->GetY();
		STEER = LEFT_JOYSTICK->GetX();

		MOTOR_LM->Set(ControlMode::PercentOutput, THROTTLE+STEER);
		MOTOR_RM->Set(ControlMode::PercentOutput, -THROTTLE+STEER);

		//OUTPUT = VOLTAGE_CONVERT(AUTO_RANGEFINDER->GetVoltage());
		SmartDashboard::PutNumber("RPMS_L",  MOTOR_ENCODER_L->GetRate());
		SmartDashboard::PutNumber("RPMS_R",  MOTOR_ENCODER_R->GetRate());
		SmartDashboard::PutNumber("CURRENT_GRYO", CONTROL_GYRO->GetAngle());
		SmartDashboard::PutNumber("RANGE", AUTO_RANGEFINDER->GetVoltage());
		SmartDashboard::PutNumber("RANGE_METERS", OUTPUT);
		SmartDashboard::PutNumber("X", INTERNAL_ACCELEROMETER->GetX());
		SmartDashboard::PutNumber("Y", INTERNAL_ACCELEROMETER->GetY());
		SmartDashboard::PutNumber("Z", INTERNAL_ACCELEROMETER->GetZ());
		frc::Wait(0.005);
	}

	void TestPeriodic() {

	}

private:

};

START_ROBOT_CLASS(Robot)
