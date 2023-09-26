/*
 * ClearPathMC.h
 *
 * Created: 9/21/2023 6:04:25 PM
 *  Author: Luke Strohbehn
 */

#include "ClearCore.h"


#ifndef CLEARPATHMC_H_
#define CLEARPATHMC_H_

// To enable automatic fault handling, #define HANDLE_MOTOR_FAULTS (1)
// To disable automatic fault handling, #define HANDLE_MOTOR_FAULTS (0)
#define HANDLE_MOTOR_FAULTS (0)

class ClearPathMC {
	private:
		// Motor
		MotorDriver motor = ConnectorM0;
		
		// A reference to the maximum clockwise and counter-clockwise velocities set in
		// the MSP software. These must match the values in MSP software
		const int32_t max_velocity_CW = 2500;
		const int32_t max_velocity_CCW = 2500;
		
		// Each velocity commanded will be a multiple of this value, which must match
		// the Velocity Resolution value in MSP. Use a lower value here (and in MSP) to
		// command velocity with a finer resolution
		const double velocity_resolution = 1.0;
		
		bool check_for_faults();
		void handle_motor_faults();
		void assert_HLFB();
		
	public:
		ClearPathMC();
		~ClearPathMC();
		
		double target_velocity = 0.0;
		double current_velocity = 0.0;
		
		void begin();
		void get_position();
		void get_velocity();
		void set_position(double pos);
		void set_velocity(double vel);
		
		void move_at_target_velocity();
		
		void stop();
};



#endif /* CLEARPATHMC_H_ */