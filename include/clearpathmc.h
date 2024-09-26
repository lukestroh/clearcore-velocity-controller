/*
 * ClearPathMC.h
 *
 * Created: 9/21/2023 6:04:25 PM
 * Author: Luke Strohbehn
 */

#include "ClearCore.h"
#include "EthUDP.h"
#include "system.h"



#ifndef CLEARPATHMC_H_
#define CLEARPATHMC_H_

#ifndef __CPMC_DEBUG__
#define __CPMC_DEBUG__ 0
#endif

// To enable automatic fault handling, #define HANDLE_MOTOR_FAULTS (1)
// To disable automatic fault handling, #define HANDLE_MOTOR_FAULTS (0)
#define HANDLE_MOTOR_FAULTS (0)

class ClearPathMC {	
	private:		
		// Motor
		MotorDriver& motor = ConnectorM0;
		
		// A reference to the maximum clockwise and counter-clockwise velocities set in
		// the MSP software. These must match the values in MSP software. DO NOT CHANGE UNLESS THIS IS ALSO CHANGED.
		const int32_t m_max_velocity = 1000;
		const int8_t m_calibration_velocity = -100;
		
		// Each velocity commanded will be a multiple of this value, which must match
		// the Velocity Resolution value in MSP. Use a lower value here (and in MSP) to
		// command velocity with a finer resolution
		const double velocity_resolution = 1.0;
		int motor_id;
		
		bool check_for_faults();
		void handle_motor_faults();
		void assert_HLFB();
		
	public:
		ClearPathMC();
		ClearPathMC(int _id);
		~ClearPathMC();

		// Limit switch pins
		DigitalIn& limit_switch_pin_neg = ConnectorDI7;
		DigitalIn& limit_switch_pin_pos = ConnectorDI8;
		// Emergency stop pin
		DigitalIn& emergency_stop_pin = ConnectorDI6;
		
		// Motor state structs
		slidersystem::DataInterface command_;
		slidersystem::DataInterface state_;
		
		double target_velocity = 0.0;
		
		void begin();
		void get_position();
		float get_velocity();
		void set_velocity(int vel);
		void set_standby();
		
		void move_at_target_velocity();
		void calibrate();
		
		//void stop();
};



#endif /* CLEARPATHMC_H_ */