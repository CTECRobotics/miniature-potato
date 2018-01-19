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
	enum {
		SEG_1 = 0,
		SEG_2 = 1,
		SEG_3 = 3,
	};
		//REGULATORY VALUES
	bool IS_TRACKING;
	bool DRIVESTATE;
	int AUTO_MODE;
	int SWITCH_1;
	int SWITCH_2;
	int SWITCH_POS;
		//CONTROL VALUES
	double CAMERA_ERROR;	//POSTIVE IS NEGATIVE, RIGHT IS POSITIVE
	double SETPOINT_GYRO;
	double NAVX_GRYO;		//OUTPUT FROM THE NAVX
	double ADX_GRYO;		//OUTPUT FROM THE SPI GYROSCOPE
	double COMPOSITE_GYRO;	//AVERAGED VALUE OF GYROSCOPES
	double L_VELOCITY;		//VELOCITY OF THE ENCODERS IN TICKS
	double R_VELOCITY;
	double ELEV_POSITION;
	double THROTTLE;
	double STEER;
	double UNIV_A_SET;		//UNIVERSAL ANGLE TARGET
	double UNIV_D_SET;		//UNIVERSAL DISTANCE TARGET
	double AUTO_RANGE;
	string ALLIANCE;
	string ALLIED_POS;
	string ALLIED_POS_INIT;
		//DON'T TOUCH ME VALUES
	const int VALVE_WAIT = 0.25;
	double ARRAY_WP [2]= {0, 0};
	const int ELEV_SCALE = 1;
	const int WINCH_SCALE = 1;
		//TEMP
	double DISTANCE;
		//CANTALON OBJECTS
	TalonSRX *MOTOR_LM;
	TalonSRX *MOTOR_LS;
	TalonSRX *MOTOR_RM;
	TalonSRX *MOTOR_RS;
	TalonSRX *WINCH_ELEV_1;
	TalonSRX *WINCH_ELEV_2;
	TalonSRX *ELEVATOR_ELEV_1;
	TalonSRX *ELEVATOR_ELEV_2;
	TalonSRX *ACTUATOR_1;
	TalonSRX *ACTUATOR_2;
	TalonSRX *BOX_LIFT;
		//SOLENOIDS
	DoubleSolenoid *GEARBOX_L;
	DoubleSolenoid *GEARBOX_R;
		//SENSOR INPUTS
	Encoder *MOTOR_ENCODER_L;
	Encoder *MOTOR_ENCODER_R;
	Encoder *WINCH_ELEV;
	ADXRS450_Gyro *CONTROL_GYRO;
	DigitalInput *TYPE_1;
	DigitalInput *TYPE_2;
	DigitalInput *TYPE_3;
		//MISC
	AHRS *PHYSIX;
	Timer *AUTO_TIMER;
	Joystick *LEFT_JOYSTICK;
	Joystick *RIGHT_JOYSTICK;

	shared_ptr<nt::NetworkTableInstance> VISION_DATA_1;
	shared_ptr<nt::NetworkTable> VISION_DATA_2;
	shared_ptr<nt::NetworkTable> VISION_DATA;

	AnalogInput *RIO_ULTRASONIC;
	Accelerometer *INTERNAL_ACCELEROMETER;
	DriverStation *DS;

	void RobotInit() {
				//VALUE SETUP
			//REGULATORY VALUES
		IS_TRACKING = false;
		DRIVESTATE = true;
		AUTO_MODE = 0;
		SWITCH_1 = 0;
		SWITCH_2 = 0;
		SWITCH_POS = 0;
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
		AUTO_RANGE= 0;
		ALLIANCE = "";
		ALLIED_POS = "";
			//TEMP
		DISTANCE = 0;

				//PERIPHERAL AND CONTROL SETUP
			//TALONSRX
		MOTOR_LM = new TalonSRX(1);
		MOTOR_LS = new TalonSRX(2);
		MOTOR_RM = new TalonSRX(3);
		MOTOR_RS = new TalonSRX(4);
		WINCH_ELEV_1 = new TalonSRX(5);
		WINCH_ELEV_2 = new TalonSRX(6);
		ELEVATOR_ELEV_1 = new TalonSRX(7);
		ELEVATOR_ELEV_2 = new TalonSRX(8);
		ACTUATOR_1 = new TalonSRX(9);
		ACTUATOR_2 = new TalonSRX(10);
		BOX_LIFT = new TalonSRX(11);
			//SOLENOIDS
		GEARBOX_L = new DoubleSolenoid(1, 1, 2);
		GEARBOX_R = new DoubleSolenoid(2, 1, 2);
			//NAVX CONNECTION ATTEMPT
        try {
            PHYSIX = new AHRS(SPI::Port::kMXP);
        } catch (std::exception ex ) {
            string ERROR_STRING = "Error instantiating navX-MXP: PHYSIX";
            ERROR_STRING += ex.what();
            DriverStation::ReportError(ERROR_STRING.c_str());
        }
        	//SENSORS, JOYSTICKS, ACCELEROMETERS, ETC.
		LEFT_JOYSTICK = new Joystick(0);
		CONTROL_GYRO = new ADXRS450_Gyro();
		RIO_ULTRASONIC = new AnalogInput(0);
		AUTO_TIMER = new Timer();
		INTERNAL_ACCELEROMETER = new BuiltInAccelerometer();
			//INITIAL SETPOINTS, CALIB, ETC.
				//ControlMode::(mode) IS NOW USED TO DETERMINE CONTROL METHOD
		MOTOR_LM->Set(ControlMode::PercentOutput, 0);
		MOTOR_RM->Set(ControlMode::PercentOutput, 0);
		MOTOR_LS->Set(ControlMode::Follower, 1);
		MOTOR_RS->Set(ControlMode::Follower, 3);
		WINCH_ELEV_1->Set(ControlMode::PercentOutput, 0);
		WINCH_ELEV_2->Set(ControlMode::Follower, 5);
		ELEVATOR_ELEV_1->Set(ControlMode::PercentOutput, 0);
		ELEVATOR_ELEV_2->Set(ControlMode::Follower, 7);
		ACTUATOR_1->Set(ControlMode::PercentOutput, 0);
		ACTUATOR_2->Set(ControlMode::Follower, 0);
		ACTUATOR_2->SetInverted(true);
		BOX_LIFT->Set(ControlMode::PercentOutput, 0);
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

		ELEVATOR_ELEV_1->ConfigForwardSoftLimitThreshold(10000, 10);
		ELEVATOR_ELEV_2->ConfigForwardSoftLimitThreshold(10000, 10);
		ELEVATOR_ELEV_1->ConfigReverseSoftLimitThreshold(-10000, 10);
		ELEVATOR_ELEV_2->ConfigReverseSoftLimitThreshold(-10000, 10);

		ELEVATOR_ELEV_1->ConfigForwardSoftLimitEnable(true, 10);
		ELEVATOR_ELEV_2->ConfigForwardSoftLimitEnable(true, 10);
		ELEVATOR_ELEV_1->ConfigReverseSoftLimitEnable(true, 10);
		ELEVATOR_ELEV_2->ConfigReverseSoftLimitEnable(true, 10);
		NetworkTable::SetServerMode();
		NetworkTable::SetIPAddress("roborio-6445-frc.local");
		NetworkTable::Initialize();

		shared_ptr<nt::NetworkTable> VISION_DATA = NetworkTable::GetTable("JETSON");
				//SWITCH SETUP
		if(TYPE_1->Get()) {
			SWITCH_POS = 1;
		} else if(TYPE_2->Get()) {
			SWITCH_POS = 2;
		} else {
			SWITCH_POS = 3;
		}
	}
	//INPUT A TURN ANGLE AND ROTATE THE ROBOT AS SUCH
	//OUTPUT A SUCCESS STATEMENT
	bool TURN_TO_ANGLE (int UNIV_A_SET ) {
		if(COMPOSITE_GYRO < (UNIV_A_SET - 2)){
			MOTOR_LM->Set(ControlMode::Velocity, -0.5);
			MOTOR_RM->Set(ControlMode::Velocity, -0.5);
		} else if(COMPOSITE_GYRO > (UNIV_A_SET + 2)) {
			MOTOR_LM->Set(ControlMode::Velocity, 0.5);
			MOTOR_RM->Set(ControlMode::Velocity, 0.5);
		} else {
			MOTOR_LM->Set(ControlMode::Velocity, 0.0);
			MOTOR_RM->Set(ControlMode::Velocity, 0.0);
		}
		return true;
	}
	bool DRIVE_TO_DISTANCE(int UNIV_D_SET, int UNIV_A_SET) {
		if((CONTROL_GYRO->GetAngle() <= (UNIV_A_SET - 2)) && DISTANCE < UNIV_D_SET) {
			MOTOR_LM->Set(ControlMode::Velocity, 0.15);
			MOTOR_RM->Set(ControlMode::Velocity, -0.15);
		} else if((CONTROL_GYRO->GetAngle() >= (UNIV_A_SET + 2)) && DISTANCE < UNIV_D_SET) {
			MOTOR_LM->Set(ControlMode::Velocity, 0.15);
			MOTOR_RM->Set(ControlMode::Velocity, -0.15);
		} else if((CONTROL_GYRO->GetAngle() > (UNIV_A_SET - 2) || CONTROL_GYRO->GetAngle() < (UNIV_A_SET + 2)) && DISTANCE < UNIV_D_SET) {
			MOTOR_LM->Set(ControlMode::Velocity, 0.15);
			MOTOR_RM->Set(ControlMode::Velocity, -0.15);
		} else {
			MOTOR_LM->Set(ControlMode::Velocity, 0.15);
			MOTOR_RM->Set(ControlMode::Velocity, -0.15);
		}
		return true;
	}
	void SWITCH_SCALE (string POSITION) {
		boolean LLL = false;
		boolean RRR = false;
		boolean LRL = false;
		boolean RLR = false;
		if(ALLIED_POS_INIT.find("LLL")){
			LLL = true;
		} else if((ALLIED_POS_INIT.find("RRR")){
			RRR = true;
		} else if(ALLIED_POS_INIT.find("LRL"){
			LRL = true;
		} else if(ALLIED_POS_INIT.find("RLR"){
			RLR = true;
		}
		if(LLL||RRR||LRL||RLR){
			if(LLL){
			SWITCH_1 = 0;
			SWITCH_2 = 0;
			}
			if(RRR){
			SWITCH_1 = 1;
			SWITCH_2 = 1;
			}
			if(LRL){
			SWITCH_1 = 0;
			SWITCH_2 = 0;
			}
			if(RLR){
			SWITCH_1 = 1;
			SWITCH_2 = 1;
			}
		}
		switch (SWITCH_1) {
		case 0:
			switch (SWITCH_2) {
			case 0:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case 1:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case 2:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			}
			break;
		case 1:
			switch (SWITCH_2) {
			case 0:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case 1:
				Segments[0]->DIST = 1;
				Segments[0]->ANGLE = 90;
				Segments[1]->DIST = 2;
				Segments[1]->ANGLE = 270;
				break;
			case 2:
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
	void AutonomousInit() override {
		ALLIANCE = std::to_string(DS->GetInstance().GetAlliance());
		SmartDashboard::PutString("ALLIANCE_COLOR", "ALLIANCE");
		ALLIED_POS = DriverStation::GetInstance().GetGameSpecificMessage();
		SWITCH_SCALE(DriverStation::GetInstance().GetGameSpecificMessage());

		AUTO_TIMER->Reset();
		AUTO_TIMER->Start();
		PHYSIX->Reset();
		CONTROL_GYRO->Reset();
	}

	void AutonomousPeriodic() {
		AUTO_MODE = VISION_DATA->GetNumber("AUTO_MODE", 0);
		NAVX_GRYO = PHYSIX->GetAngle() + 180;
		ADX_GRYO = CONTROL_GYRO->GetAngle() + 180;
		COMPOSITE_GYRO = ((NAVX_GRYO + ADX_GRYO)/2);
		AUTO_RANGE = RIO_ULTRASONIC->GetAverageValue();
		//SELECTION BASED ON SWITCH POSTION, OR ROBOT POSITION
		//IS_TRACKING WILL NEED TO BE RETRIEVED FROM NETWORKTABLES
		//TIME WILL NEED TO BE SENT TO JEFF ALONG WITH CURRENT DISTANCE
		if(!IS_TRACKING) {
			if(DRIVESTATE) {
				DRIVE_TO_DISTANCE(Segments[0]->DIST, Segments[0]->ANGLE);
			} else {
				TURN_TO_ANGLE(Segments[1]->ANGLE);
			}
		} else if (IS_TRACKING) {

		}
	}

	void TeleopInit() {
		AUTO_TIMER->Stop();
		PHYSIX->Reset();
		CONTROL_GYRO->Reset();
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

		NAVX_GRYO = PHYSIX->GetAngle();
		ADX_GRYO = CONTROL_GYRO->GetAngle();
		COMPOSITE_GYRO = ((NAVX_GRYO + ADX_GRYO)/2);

		L_VELOCITY = MOTOR_LM->GetSelectedSensorVelocity(1); //*SOME SCALE FACTOR
		R_VELOCITY = MOTOR_RM->GetSelectedSensorVelocity(3);

		RIO_ULTRASONIC->GetAverageValue();

		SmartDashboard::PutNumber("RPMS_L",  MOTOR_ENCODER_L->GetRate());
		SmartDashboard::PutNumber("RPMS_R",  MOTOR_ENCODER_R->GetRate());
		SmartDashboard::PutNumber("ADX_OUTPUT", CONTROL_GYRO->GetAngle());
		SmartDashboard::PutNumber("NAVX_OUTPUT", PHYSIX->GetAngle());
		SmartDashboard::PutNumber("COMPOSITE_OUTPUT", COMPOSITE_GYRO);
		SmartDashboard::PutNumber("RANGE", RIO_ULTRASONIC->GetVoltage());
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
