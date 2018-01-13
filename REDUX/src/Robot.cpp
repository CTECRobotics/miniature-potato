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
	double SETPOINT_GYRO;
	double NAVX_GRYO;		//OUTPUT FROM THE NAVX
	double ADX_GRYO;		//OUTPUT FROM THE SPI
	double COMPOSITE_GYRO;	//AVERAGED VALUE OF GYROSCOPES
	double L_VELOCITY;		//VELOCITY OF THE ENCODERS IN TICKS
	double R_VELOCITY;
	double ELEV_POSITION;
	double THROTTLE;
	double STEER;
	double UNIV_A_SET;		//UNIVERSAL
	std::string ALLIANCE;
	std::string ALLIED_POS;
		//DON'T TOUCH ME VALUES
	double MAXIMUM_RPM;
	double VALVE_WAIT;
	double ARRAY_WP_1 [2]= {5, 90};
	double ARRAY_WP_2 [2]= {5, 90};
	double ARRAY_WP_3 [2]= {5, 90};
	const int ELEV_SCALE = 1;
	const int WINCH_SCALE = 1;
		//MISC


		//CANTALON OBJECTS
	TalonSRX *MOTOR_LM;
	TalonSRX *MOTOR_LS;
	TalonSRX *MOTOR_RM;
	TalonSRX *MOTOR_RS;
	TalonSRX *WINCH_ELEV_1;
	TalonSRX *WINCH_ELEV_2;
	TalonSRX *ACTUATOR_1;
	TalonSRX *ACTUATOR_2;
		//PID ASSESTS
		//PID LOOPS
	PIDController *MOTOR_L_PID;
	PIDController *MOTOR_R_PID;
		//SOLENOIDS
	DoubleSolenoid *GEARBOX_L;
	DoubleSolenoid *GEARBOX_R;
	DoubleSolenoid *ELEV_WINCH;
	DoubleSolenoid *ARM_L;
	DoubleSolenoid *ARM_R;
		//SENSOR INPUTS
	Encoder *MOTOR_ENCODER_L;
	Encoder *MOTOR_ENCODER_R;
	Encoder *WINCH_ELEV;
	ADXRS450_Gyro *CONTROL_GYRO;
		//MISC
	AHRS *PHYSIX;
	Timer *AUTO_TIMER;
	Joystick *LEFT_JOYSTICK;
	Joystick *RIGHT_JOYSTICK;
	std::shared_ptr<nt::NetworkTable> VISION_DATA;
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
		SETPOINT_GYRO = 0;
		NAVX_GRYO = 0;
		ADX_GRYO = 0;
		COMPOSITE_GYRO = 0;
		L_VELOCITY = 0;
		R_VELOCITY = 0;
		THROTTLE = 0;
		STEER = 0;
		UNIV_A_SET = 0;		//THIS IS THE UNIVERSAL ANGLE SETPOINT FOR AUTO
//		ALLIANCE = "";
//		ALLIED_POS = "";
			//DO NOT MODIFY
		MAXIMUM_RPM = 511;
		VALVE_WAIT = 0.25;

				//PERIPHERAL AND CONTROL SETUP
			//TALONSRX
		MOTOR_LM = new TalonSRX(1);
		MOTOR_LS = new TalonSRX(2);
		MOTOR_RM = new TalonSRX(3);
		MOTOR_RS = new TalonSRX(4);
		WINCH_ELEV_1 = new TalonSRX(5);
		WINCH_ELEV_2 = new TalonSRX(6);
			//SOLENOIDS
		GEARBOX_L = new DoubleSolenoid(1, 1, 1);
		GEARBOX_R = new DoubleSolenoid(1, 1, 1);
		ELEV_WINCH = new DoubleSolenoid(1, 1, 1);
		ARM_L = new DoubleSolenoid(1, 1, 1);
		ARM_R = new DoubleSolenoid(1, 1, 1);

//			//NAVX CONNECTION ATTEMPT
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
				//ControlMode::(mode) IS NOW USED TO DETERMINE CONTROL METHOD
		MOTOR_LM->Set(ControlMode::PercentOutput, 0);
		MOTOR_RM->Set(ControlMode::PercentOutput, 0);
		MOTOR_LS->Set(ControlMode::PercentOutput, 0);
		MOTOR_RS->Set(ControlMode::PercentOutput, 0);
		MOTOR_LS->Set(ControlMode::Follower, 1);
		MOTOR_RS->Set(ControlMode::Follower, 3);
		WINCH_ELEV_1->Set(ControlMode::PercentOutput, 0);
		WINCH_ELEV_2->Set(ControlMode::PercentOutput, 0);
		WINCH_ELEV_2->Set(ControlMode::Follower, 5);
		ACTUATOR_1->Set(ControlMode::PercentOutput, 0);
		ACTUATOR_2->Set(ControlMode::PercentOutput, 0);
		CONTROL_GYRO->Calibrate();
			//MISC
			//INTERNAL PID SETUP
		MOTOR_LM->GetSelectedSensorVelocity(1);
				//SELECTS SENSOR BASED OFF CHANNEL
		MOTOR_LM->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
				//SETS UP SENSOR TYPE, PID SLOT (DEFINED AS pidIdx) NUMBER, TIMEOUT
		MOTOR_LM->SetSensorPhase(true);
				//DETERMINES READ DIRECTIONS

		MOTOR_LM->ConfigNominalOutputForward(0, 10);
		MOTOR_LM->ConfigNominalOutputReverse(0, 10);
		MOTOR_LM->ConfigPeakOutputForward(1, 10);
		MOTOR_LM->ConfigPeakOutputReverse(-1, 10);

				//SETS UP THE VARIOUS VALUES IN THE PID SLOT
		MOTOR_LM->Config_kF(0, 0.0, 10);
		MOTOR_LM->Config_kP(0, 0.1, 10);
		MOTOR_LM->Config_kI(0, 0.0, 10);
		MOTOR_LM->Config_kD(0, 0.0, 10);

		MOTOR_RM->GetSelectedSensorVelocity(2);
		MOTOR_RM->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		MOTOR_RM->SetSensorPhase(true);

		MOTOR_RM->ConfigNominalOutputForward(0, 10);
		MOTOR_RM->ConfigNominalOutputReverse(0, 10);
		MOTOR_RM->ConfigPeakOutputForward(1, 10);
		MOTOR_RM->ConfigPeakOutputReverse(-1, 10);

		MOTOR_RM->Config_kF(0, 0.0, 10);
		MOTOR_RM->Config_kP(0, 0.1, 10);
		MOTOR_RM->Config_kI(0, 0.0, 10);
		MOTOR_RM->Config_kD(0, 0.0, 10);

		VISION_DATA = nt::NetworkTable::GetTable("JETSON");

	}
	void TURN_TO_ANGLE (int UNIV_A_SET ) {
		if(COMPOSITE_GYRO < UNIV_A_SET ){
			MOTOR_LM->Set(ControlMode::Velocity, -0.5);
			MOTOR_RM->Set(ControlMode::Velocity, 0.5);
		} else if(COMPOSITE_GYRO > UNIV_A_SET ) {

		} else {

		}

	}
	void DRIVE_TO_DISTANCE() {
		if(CONTROL_GYRO->GetAngle() <= (UNIV_A_SET - 2)) {
			MOTOR_LM->Set(ControlMode::PercentOutput, 0.15);
			MOTOR_RM->Set(ControlMode::PercentOutput, -0.08);
		} else if(CONTROL_GYRO->GetAngle() >= (UNIV_A_SET + 2)) {
			MOTOR_LM->Set(ControlMode::PercentOutput, 0.05);
			MOTOR_RM->Set(ControlMode::PercentOutput, -0.18);
		} else if(CONTROL_GYRO->GetAngle() > (UNIV_A_SET - 2) || CONTROL_GYRO->GetAngle() < (UNIV_A_SET + 2)) {
			MOTOR_LM->Set(ControlMode::PercentOutput, 0.25);
			MOTOR_RM->Set(ControlMode::PercentOutput, -0.28);
		}
	}
	int CALC_ELEV_POSITION(int ELEVATOR_DEGREES) {
		double ELEV_POSITION = ELEVATOR_DEGREES*ELEV_SCALE;
		return ELEV_POSITION;
	}
	void SHIFT_HIGH () {
		GEARBOX_L->Set(DoubleSolenoid::kForward);
		GEARBOX_R->Set(DoubleSolenoid::kForward);
		frc::Wait(0.5);
		GEARBOX_L->Set(DoubleSolenoid::kOff);
		GEARBOX_R->Set(DoubleSolenoid::kOff);
	}
	void SHIFT_LOW () {
		GEARBOX_L->Set(DoubleSolenoid::kReverse);
		GEARBOX_R->Set(DoubleSolenoid::kReverse);
		frc::Wait(0.5);
		GEARBOX_L->Set(DoubleSolenoid::kOff);
		GEARBOX_R->Set(DoubleSolenoid::kOff);
	}
	void ACTUATOR_UP() {
		ARM_L->Set(DoubleSolenoid::kReverse);
		ARM_R->Set(DoubleSolenoid::kReverse);
		frc::Wait(0.5);
		ARM_L->Set(DoubleSolenoid::kOff);
		ARM_R->Set(DoubleSolenoid::kOff);
	}
	void ACTUATOR_DOWN () {
		ARM_L->Set(DoubleSolenoid::kForward);
		ARM_R->Set(DoubleSolenoid::kForward);
		frc::Wait(0.5);
		ARM_L->Set(DoubleSolenoid::kOff);
		ARM_R->Set(DoubleSolenoid::kOff);
	}
	void CHANGE_TO_WINCH () {
		ELEV_WINCH->Set(DoubleSolenoid::kReverse);
		frc::Wait(0.5);
		ELEV_WINCH->Set(DoubleSolenoid::kOff);
	}
	void CHANGE_TO_ELEVATOR () {
		ELEV_WINCH->Set(DoubleSolenoid::kForward);
		frc::Wait(0.5);
		ELEV_WINCH->Set(DoubleSolenoid::kOff);
	}
	void AutonomousInit() override {
		ALLIANCE = std::to_string(DS->GetInstance().GetAlliance());
		SmartDashboard::PutString("ALLIANCE_COLOR", "ALLIANCE");

		MOTOR_ENCODER_L->Reset();
		MOTOR_ENCODER_R->Reset();
		WINCH_ELEV->Reset();

		PHYSIX->Reset();

		AUTO_TIMER->Reset();
		AUTO_TIMER->Start();
		CONTROL_GYRO->Reset();
	}

	void AutonomousPeriodic() {
		AUTO_MODE = VISION_DATA->GetNumber("AUTO_MODE", 0);
		NAVX_GRYO = PHYSIX->GetAngle();
		ADX_GRYO = CONTROL_GYRO->GetAngle();
		COMPOSITE_GYRO = ((NAVX_GRYO + ADX_GRYO)/2);
		ALLIED_POS = DriverStation::GetInstance().GetGameSpecificMessage();
		if(ALLIED_POS[0] == 'L') {
			AUTO_MODE = 0;
		} else {
			AUTO_MODE = 1;
		}
		switch (AUTO_MODE) {
		case 0:

			break;
		case 1:

			break;
		case 2:

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
		if (LEFT_JOYSTICK->GetRawButton(1)) {
			SHIFT_HIGH();
		}
		if (LEFT_JOYSTICK->GetRawButton(2)) {
			SHIFT_LOW();
		}
		if (LEFT_JOYSTICK->GetRawButton(11)) {
			ACTUATOR_UP();
		}
		if (LEFT_JOYSTICK->GetRawButton(8)) {
			ACTUATOR_DOWN();
		}


		NAVX_GRYO = PHYSIX->GetAngle();
		ADX_GRYO = CONTROL_GYRO->GetAngle();
		COMPOSITE_GYRO = ((NAVX_GRYO + ADX_GRYO)/2);

		L_VELOCITY = MOTOR_LM->GetSelectedSensorVelocity(1); //*SOME SCALE FACTOR
		R_VELOCITY = MOTOR_RM->GetSelectedSensorVelocity(3);

//		OUTPUT = VOLTAGE_CONVERT(AUTO_RANGEFINDER->GetVoltage());
		SmartDashboard::PutNumber("RPMS_L",  MOTOR_ENCODER_L->GetRate());
		SmartDashboard::PutNumber("RPMS_R",  MOTOR_ENCODER_R->GetRate());
		SmartDashboard::PutNumber("ADX_OUTPUT", CONTROL_GYRO->GetAngle());
		SmartDashboard::PutNumber("NAVX_OUTPUT", PHYSIX->GetAngle());
		SmartDashboard::PutNumber("COMPOSITE", COMPOSITE_GYRO);
		SmartDashboard::PutNumber("RANGE", AUTO_RANGEFINDER->GetVoltage());
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
