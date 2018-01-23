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
	enum SEGMENT_STATE{
		SEG_1 = 0,
		SEG_2 = 1,
		SEG_3 = 2,
	};
	//TRACKING WHICH SEGMENT THE ROBOT IS EXECUTING
	enum SWITCH_POS_VALUES{
		LEFT_FIELD = 0,
		RIGHT_FIELD = 1,
	};
	//VALUES FOR TRACKING WHICH SIDE OF THE FIELD THE SWITCH IS ON
	enum ROBOT_STARTING_POS{
		ROB_ON_LEFT = 0,
		ROB_ON_CENTER = 1,
		ROB_ON_RIGHT = 2,
	};
	//DESIGNATES CONTROL STATES FOR AUTONOMOUS SCHEDULING
		//REGULATORY VALUES
	bool IS_TRACKING;
	bool DRIVESTATE;  //BAC: this variable seems unused
	int AUTO_MODE;
	int FIELD_POS;
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
	int DRIVE_STATE;
	string ALLIANCE;
	string ALLIED_POS;
	string ALLIED_POS_INIT;
		//DON'T TOUCH ME VALUES
	const int VALVE_WAIT = 0.25;
	double ARRAY_WP [2]= {0, 0};
	const int ELEV_SCALE = 1;
	const int WINCH_SCALE = 1;
	const int ROT_TICKS = 4096;
	const float PI = 3.1415;
	const float CIRCUMFERENCE = (PI*6.0);
	const float ULTRA_VOLT_TO_MM = 5000/4.88;

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
		FIELD_POS = 0;
			//CONTROL VALUES
		SETPOINT_GYRO = 0;
		NAVX_GRYO = 0;
		ADX_GRYO = 0;
		COMPOSITE_GYRO = 0;
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
		RIGHT_JOYSTICK = new Joystick(1);
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
		MOTOR_LM->Config_kF(0, 0.375, 10);
		MOTOR_LM->Config_kP(0, 0.125, 10);
		MOTOR_LM->Config_kI(0, 0.05, 10);
		MOTOR_LM->Config_kD(0, 0.05, 10);

		MOTOR_RM->GetSelectedSensorVelocity(2);
		MOTOR_RM->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		MOTOR_RM->SetSensorPhase(true);
		MOTOR_RM->ConfigNominalOutputForward(0, 10);
		MOTOR_RM->ConfigNominalOutputReverse(0, 10);
		MOTOR_RM->ConfigPeakOutputForward(1, 10);
		MOTOR_RM->ConfigPeakOutputReverse(-1, 10);
		MOTOR_RM->Config_kF(0, 0.375, 10);
		MOTOR_RM->Config_kP(0, 0.125, 10);
		MOTOR_RM->Config_kI(0, 0.05, 10);
		MOTOR_RM->Config_kD(0, 0.05, 10);

		ELEVATOR_ELEV_1->ConfigForwardSoftLimitThreshold(10000, 10);
		ELEVATOR_ELEV_2->ConfigForwardSoftLimitThreshold(10000, 10);
		ELEVATOR_ELEV_1->ConfigReverseSoftLimitThreshold(-10000, 10);
		ELEVATOR_ELEV_2->ConfigReverseSoftLimitThreshold(-10000, 10);
		//SOFT LIMITS TO ENSURE THAT ELEATOR SYSTEM DOES NOT BREAK THINGS
		//UNITS ARE IN NATIVE UNITS, 4096 PER ROTATION
		ELEVATOR_ELEV_1->ConfigForwardSoftLimitEnable(true, 10);
		ELEVATOR_ELEV_2->ConfigForwardSoftLimitEnable(true, 10);
		ELEVATOR_ELEV_1->ConfigReverseSoftLimitEnable(true, 10);
		ELEVATOR_ELEV_2->ConfigReverseSoftLimitEnable(true, 10);
		NetworkTable::SetServerMode();
		//SERVER VS CLIENT MODE
		NetworkTable::SetIPAddress("roborio-6445-frc.local");
		//SETS THE ROBORIO IP ADDRESS FOR THE JETSON TX1
		NetworkTable::Initialize();
		//STARTS UP NETWORK TABLE
		shared_ptr<nt::NetworkTable> VISION_DATA = NetworkTable::GetTable("JETSON");
	}
	//INPUT A TURN ANGLE AND ROTATE THE ROBOT AS SUCH
	//OUTPUT A SUCCESS STATEMENT
	bool TURN_TO_ANGLE (int UNIV_A_SET) {
		const float INCH_TO_METRE = ((25.4)/1000.0);
		const float DRIVETRAIN_RADIUS = 12.5625;
		//DISTANCE FROM CENTER LINE OF ROBOT TO CENTER OF MIDDLE DRIVE WHEELS, WEST COAST STYLE
		//4096 "TICKS" PER ROTATION, AS COUNTED BY THE ENCODERS
		const int ANGLE_TOLERANCE = 2;
		float DIFFERENCE = 0;
		double DIRECTION;
		float DISTANCE_PER_WHEEL = 0;
		NAVX_GRYO = PHYSIX->GetAngle() + 180;
		ADX_GRYO = CONTROL_GYRO->GetAngle() + 180;
		COMPOSITE_GYRO = ((NAVX_GRYO + ADX_GRYO)/2);
		bool DONE_TURNING;

		DIFFERENCE = ((abs(UNIV_A_SET - COMPOSITE_GYRO)) * (PI/180));
			//ANGULAR DIFFERENCE IN RADIANS
		DISTANCE_PER_WHEEL = (DRIVETRAIN_RADIUS * INCH_TO_METRE * DIFFERENCE);
			//DISTANCE TO TRAVEL PER WHEEL IN METRES
			//Essentially DIST = RADIUS * THETA
		DIRECTION = ((sin((UNIV_A_SET - COMPOSITE_GYRO)))/abs(sin(COMPOSITE_GYRO - UNIV_A_SET)));


		if((COMPOSITE_GYRO != (UNIV_A_SET - ANGLE_TOLERANCE)) || (COMPOSITE_GYRO != (UNIV_A_SET + ANGLE_TOLERANCE))) {
			MOTOR_LM->Set(ControlMode::Position, (DISTANCE_PER_WHEEL/CIRCUMFERENCE) * ROT_TICKS * DIRECTION);
			MOTOR_RM->Set(ControlMode::Position, (DISTANCE_PER_WHEEL/CIRCUMFERENCE) * ROT_TICKS * DIRECTION);
			DONE_TURNING = false;
		} else {
			MOTOR_LM->Set(ControlMode::Velocity, 0.0);
			MOTOR_RM->Set(ControlMode::Velocity, 0.0);
			DONE_TURNING = true;
		}
return DONE_TURNING;
	}

	bool DRIVE_TO_DISTANCE(int UNIV_D_SET) {
		//TODO
		//CONVERT FIELD DIMENSIONS TO METRES
		const int TOLERANCE = 1024;
		bool DONE_DRIVING;

		if((MOTOR_LM->GetSelectedSensorPosition(1) < UNIV_D_SET - TOLERANCE) && (MOTOR_RM->GetSelectedSensorPosition(1) < UNIV_D_SET - TOLERANCE)) {
			MOTOR_LM->Set(ControlMode::Position, (UNIV_D_SET/CIRCUMFERENCE) * ROT_TICKS);
			MOTOR_RM->Set(ControlMode::Position, (UNIV_D_SET/CIRCUMFERENCE) * ROT_TICKS);
			DONE_DRIVING = false;
		} else {
			MOTOR_LM->Set(ControlMode::Velocity, 0.0);
			MOTOR_RM->Set(ControlMode::Velocity, 0.0);
			DONE_DRIVING = true;
		}
		return DONE_DRIVING;

		//AND FIX THIS LATER, PLEASE
	}

	//BAC: I suggest a different name for this function.
	//It sets up the segments for the autonomous mode distance
	// and turn, based on which location we are at on the field,
	//and based on where on the field the robot is.
	//Suggest something like: AutoSegmentsSetup
	void SWITCH_SCALE (string POSITION) {
		bool LLL = false;
		bool RRR = false;
		bool LRL = false;
		bool RLR = false;
		int SWITCH_POS = 0;
		ALLIED_POS_INIT = POSITION;
		if(ALLIED_POS_INIT.find("LLL") != string::npos) {
			LLL = true;
			//TODO
			//npos CORRESPONDS TO "NOTHING FOUND", FLIP LOGIC?
		} else if(ALLIED_POS_INIT.find("RRR") == string::npos) {
			RRR = true;
		} else if(ALLIED_POS_INIT.find("LRL") == string::npos) {
			LRL = true;
		} else if(ALLIED_POS_INIT.find("RLR") == string::npos) {
			RLR = true;
		}
		if(LLL||RRR||LRL||RLR){ //BAC: This If-statement is unecessary
			//because all of the cases are covered below. I suggest
			//removing the first if-statement, and then adding an Else case at the very end
			if(LLL){
			SWITCH_POS = 0;
			}
			if(RRR){
			SWITCH_POS = 1;
			}
			if(LRL){
			SWITCH_POS = 0;
			}
			if(RLR){
			SWITCH_POS = 1;
			}
			//BAC: suggest adding an else case
		}
		switch (SWITCH_POS) {
		//SWITCH_POS is the variable determining switch position, left or right
		case LEFT_FIELD:
			switch (FIELD_POS) {
			//FIELD_POS is the variable determining robot position on the field , left, center, middle
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
			switch (FIELD_POS) {
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
		ALLIANCE = to_string(DS->GetInstance().GetAlliance());
		SmartDashboard::PutString("ALLIANCE_COLOR", "ALLIANCE");
		ALLIED_POS = DriverStation::GetInstance().GetGameSpecificMessage();
		SWITCH_SCALE(ALLIED_POS);
		//BAC: ALLIED_POS_INIT is a global variable used for the same
		//purpose, in the SWITCH_SCALE function. Suggest having just one variable
		//The variable may not need to be global, either:
		//we get the value here in AutoInit, then we can just pass it
		// to SWITCH_SCALE. Then I don't think it's used after that.
		SHIFT_HIGH();
		DRIVE_STATE = SEG_1;

		AUTO_TIMER->Reset();
		AUTO_TIMER->Start();
		PHYSIX->Reset();
		CONTROL_GYRO->Reset();
	}

	void AutonomousPeriodic() {
		AUTO_RANGE = RIO_ULTRASONIC->GetAverageValue();
		//SELECTION BASED ON SWITCH POSTION, OR ROBOT POSITION
		//IS_TRACKING WILL NEED TO BE RETRIEVED FROM NETWORKTABLES
		//TIME WILL NEED TO BE SENT TO JEFF ALONG WITH CURRENT DISTANCE
		float ANGEL;
		VISION_DATA->PutNumber("TIME", AUTO_TIMER->Get());
		VISION_DATA->PutNumber("DISTANCE", ULTRA_VOLT_TO_MM * RIO_ULTRASONIC->GetAverageVoltage());
		ANGEL = VISION_DATA->GetNumber("ANGLE", -1);
		IS_TRACKING = VISION_DATA->GetBoolean("TRACKING", false);

		bool DONE_DRIVING = false;
		bool DONE_TURNING = false;

		if(!IS_TRACKING) {
			DONE_DRIVING = DRIVE_TO_DISTANCE(Segments[DRIVE_STATE]->DIST);
			if (DONE_DRIVING) {
				DONE_TURNING = TURN_TO_ANGLE(Segments[DRIVE_STATE]->ANGLE);
			}
			if (DONE_DRIVING && DONE_TURNING) {
				DRIVE_STATE ++;
			}
		} else {
			DONE_TURNING = TURN_TO_ANGLE(ANGEL);
			if(DONE_TURNING) {
				DRIVE_TO_DISTANCE(ULTRA_VOLT_TO_MM * RIO_ULTRASONIC->GetAverageValue() - 0.125);
			} else {
//TODO FIND SOMETHING FOR HERE OR
			}
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
		if(LEFT_JOYSTICK->GetRawButton(10)) {
			ELEVATOR_ELEV_1->Set(ControlMode::Disabled, 0);
			ELEVATOR_ELEV_2->Set(ControlMode::Disabled, 0);
			//ELEVATOR_ELEV_1->boo
			WINCH_ELEV_1->Set(ControlMode::Velocity, RIGHT_JOYSTICK->GetY());
			WINCH_ELEV_2->Set(ControlMode::Velocity, RIGHT_JOYSTICK->GetY());
		}
		if(LEFT_JOYSTICK->GetRawAxis(9)) {
			ELEVATOR_ELEV_1->Set(ControlMode::Velocity, RIGHT_JOYSTICK->GetY());
			ELEVATOR_ELEV_2->Set(ControlMode::Velocity, RIGHT_JOYSTICK->GetY());
			WINCH_ELEV_1->Set(ControlMode::Disabled, 0);
			WINCH_ELEV_2->Set(ControlMode::Disabled, 0);
		}
		ELEVATOR_ELEV_1->Set(ControlMode::Velocity, 10);
		NAVX_GRYO = PHYSIX->GetAngle();
		ADX_GRYO = CONTROL_GYRO->GetAngle();
		COMPOSITE_GYRO = ((NAVX_GRYO + ADX_GRYO)/2);

		L_VELOCITY = MOTOR_LM->GetSelectedSensorVelocity(1); //*SOME SCALE FACTOR
		R_VELOCITY = MOTOR_RM->GetSelectedSensorVelocity(3);

		RIO_ULTRASONIC->GetAverageValue();

		SmartDashboard::PutNumber("RPMS_L", L_VELOCITY);
		SmartDashboard::PutNumber("RPMS_R", R_VELOCITY);
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
