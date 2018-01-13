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
	double THROTTLE;
	double STEER;
	TalonSRX *M_L;
	TalonSRX *M_R;
	TalonSRX *M_LL;
	TalonSRX *M_RR;
	Joystick *L;
	ADXRS450_Gyro *SPI_GYRO;
	AHRS *PHYSIX;
	void RobotInit() {
		THROTTLE = 0;
		STEER = 0;
		M_L = new TalonSRX(1);
		M_R = new TalonSRX(3);
		M_LL = new TalonSRX(2);
		M_RR = new TalonSRX(4);
		L = new Joystick(1);
		SPI_GYRO= new ADXRS450_Gyro();
        try {
            PHYSIX = new AHRS(SPI::Port::kMXP);
        } catch (std::exception ex ) {
            std::string ERROR_STRING = "Error instantiating navX-MXP: PHYSIX";
            ERROR_STRING += ex.what();
            DriverStation::ReportError(ERROR_STRING.c_str());
        }
		M_L->Set(ControlMode::PercentOutput, 0);
		M_R->Set(ControlMode::PercentOutput, 0);
		M_LL->Set(ControlMode::PercentOutput, 0);
		M_RR->Set(ControlMode::PercentOutput, 0);
		SPI_GYRO->Reset();
		SPI_GYRO->Calibrate();
	}
	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
	//	PHYSIX->Reset();
	}

	void TeleopPeriodic() {
		THROTTLE = L->GetY();
		STEER = L->GetX();
		M_L->Set(ControlMode::PercentOutput, THROTTLE + STEER);
		M_R->Set(ControlMode::PercentOutput, -THROTTLE + STEER);
		M_LL->Set(ControlMode::PercentOutput, THROTTLE + STEER);
		M_RR->Set(ControlMode::PercentOutput, -THROTTLE + STEER);

		SmartDashboard::PutNumber("GYRO DATA", SPI_GYRO->GetAngle());
		SmartDashboard::PutNumber("NAVX DATA", PHYSIX->GetAngle());
		SmartDashboard::PutNumber("ACCEL Y NAVX", PHYSIX->GetRawAccelX());
		SmartDashboard::PutNumber("ACCEL Y NAVX", PHYSIX->GetVelocityX());

		if(L->GetRawButton(5)) {
			SPI_GYRO->Reset();
			PHYSIX->Reset();
		}
	}

	void TestPeriodic() {}

private:
};

START_ROBOT_CLASS(Robot)
