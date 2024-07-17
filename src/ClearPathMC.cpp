/*
 * ClearPathMC.cpp
 *
 * Created: 9/25/2023 11:37:01 AM
 * Author: Luke Strohbehn
 */

#ifndef __SERIAL_DEBUG__
#define __SERIAL_DEBUG__ 0
#endif

#include "ClearPathMC.h"
#include "interrupts.h"

extern volatile bool neg_lim_switch_flag;
extern volatile bool pos_lim_switch_flag;
extern volatile bool e_stop_flag;
extern slidersystem::SystemStatus system_status;

ClearPathMC::ClearPathMC() {}


ClearPathMC::ClearPathMC(int _id): 
	motor_id(_id) 
{
	
}


ClearPathMC::~ClearPathMC() {}


void ClearPathMC::begin() {
	/* Configure motor settings for mode and HLFB */
	
	// Set all motor connectors to the correct mode for Manual Velocity mode
	MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_DIRECT);
	
	// Put the motor connector into HLFB mode to read bipolar PWM
	motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	
	// Set the HLFB carrier frequency to 482 Hz
	motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
	
	// Enforce the state of the motor's A and B inputs before enabling the motor
	motor.MotorInAState(false);
	motor.MotorInBState(false);
		
	// Enable the motor
	motor.EnableRequest(true);
#if __SERIAL_DEBUG__
	ConnectorUsb.SendLine("Motor enabled.");
#endif
	// Enable pin interrupts TODO: This is totally in the wrong place, there should be a system manager file, would also clean up main.cpp
	limit_switch_pin_neg.InterruptHandlerSet(&neg_lim_switch_callback, InputManager::LOW, true); // What if we did LOW... would it then trigger all of the time, letting us use 'read_interrupt'?
	limit_switch_pin_pos.InterruptHandlerSet(&pos_lim_switch_callback, InputManager::LOW, true);
	emergency_stop_pin.InterruptHandlerSet(&emergency_stop_callback, InputManager::RISING, true);
	
	// Wait for HLFB
	assert_HLFB();
#if __SERIAL_DEBUG__
	ConnectorUsb.SendLine("Motor setup complete.");
#endif
}


bool ClearPathMC::check_for_faults() {
	/* Check if a motor fault is currently preventing motion, clear fault if configured to do so.
	 * Returns true if in fault.
	*/
	if (motor.StatusReg().bit.MotorInFault) {
		if (HANDLE_MOTOR_FAULTS) {
			handle_motor_faults();
#if __SERIAL_DEBUG__
			ConnectorUsb.SendLine("Motor fault detected. Move canceled.");	
		}
		else {
			ConnectorUsb.SendLine("Motor fault detected. Move canceled. Enable automatic fault handling by setting HANDLE_MOTOR FAULTS to 1.");
#endif
		}
		return true;
	}
	return false;
}


void ClearPathMC::handle_motor_faults() {
	/* Clears motor faults by cycling enable to the motor.
	 *    Assumes motor is in fault 
	 *      (this function is called when motor.StatusReg.MotorInFault == true)
	 */
#if __SERIAL_DEBUG__
 	ConnectorUsb.SendLine("Handling fault: clearing faults by cycling enable signal to motor.");
#endif
	motor.EnableRequest(false);
	Delay_ms(10);
	motor.EnableRequest(true);
	Delay_ms(100);
 }
 

void ClearPathMC::assert_HLFB() {
	/* Make sure the HLFB is connected */
	while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED && !motor.StatusReg().bit.MotorInFault) {
#if __SERIAL_DEBUG__
		ConnectorUsb.SendLine("ERROR IN HLFB ASSERT:");
		ConnectorUsb.Send("\tHLFB STATE: ");
		ConnectorUsb.SendLine(motor.HlfbState());
		ConnectorUsb.Send("\tMOTOR IN FAULT: ");
		ConnectorUsb.SendLine(motor.StatusReg().bit.MotorInFault);
		
		ConnectorUsb.Send("\tHLFB Percent: ");
		ConnectorUsb.SendLine(motor.HlfbPercent());
#endif
		Delay_ms(100);
		continue;
	}
}


float ClearPathMC::get_velocity() {
	/* 
	Get the current velocity from the HLFB
	The duty cycle scales as a percentage of the maximum motor speed configured in the currently selected operating mode.
		- 5% duty cycle = 0% max speed
		- 95% duty cycle = 100% max speed
	HLFB output de-asserts (i.e., 0% duty cycle, "off", non-conducting) when the motor is disabled or shutdown.
	*/
	MotorDriver::HlfbStates hlfb_state = motor.HlfbState();
	if (hlfb_state == MotorDriver::HLFB_HAS_MEASUREMENT) {
		// Get the measured speed as a percent of Max Speed
		float hlfb_vel_percent = motor.HlfbPercent();
		float hlfb_vel = hlfb_vel_percent * m_max_velocity;
		return hlfb_vel;
	}
	else {
		return 0.0;
	}
}

void ClearPathMC::set_velocity(int vel, slidersystem::SystemStatus* system_status) {
	/* Set the target velocity of the ClearPath MC motor, according to maximum velocity limits. Commands are sent as positive RPM==positive direction (away from motor). In reality, positive RPM values drive the base to the direction of the motor. */
	
	// Check if standby or e-stop
	if (*system_status==slidersystem::E_STOP || *system_status==slidersystem::SYSTEM_STANDBY) {
		target_velocity = 0;
		return;
	}
	
	// check the limit switch statuses
	if (vel >= 0 && *system_status==slidersystem::POS_LIM){
		#if __SERIAL_DEBUG__
		//switch_name = "positive";
		ConnectorUsb.SendLine("Commanded velocity was stopped by the positive limit switch");
		#endif
		target_velocity = 0;
		return;
	}
	else if (vel <= 0 && *system_status==slidersystem::NEG_LIM){
		#if __SERIAL_DEBUG__
		//switch_name = "negative";
		ConnectorUsb.SendLine("Commanded velocity was stopped by the negative limit switch");
		#endif
		target_velocity = 0;
		return;
	}
	
	// Correct command to speed limit
	if (vel > m_max_velocity) {
		target_velocity = -1 * m_max_velocity;
	}
	else if (vel < -1 * m_max_velocity) {
		target_velocity = m_max_velocity;
	}
	else {
		target_velocity = -1 * vel;
	}
#if __SERIAL_DEBUG__
	ConnectorUsb.Send("commanded vel: ");
	ConnectorUsb.SendLine(vel);
	ConnectorUsb.Send("target vel: ");
	ConnectorUsb.SendLine(target_velocity);
	ConnectorUsb.Send("Curr vel: ");
	ConnectorUsb.SendLine(current_velocity);
	//Delay_ms(1000);
#endif
}

void ClearPathMC::set_standby() {
	set_velocity(0, &system_status);
}


void ClearPathMC::move_at_target_velocity() {
	/* Move the motor at the set target velocity */
	
	// Check motor status
	check_for_faults();
	
	// TODO: Handle this limit elsewhere?
	// If at negative limit switch, don't let target velocity be negative
	if (neg_lim_switch_flag && target_velocity < 0) {
		target_velocity = 0;
	}
	
	// If at positive limit switch, don't let target velocity be positive
	if (pos_lim_switch_flag && target_velocity > 0) {
		target_velocity = 0;
	}
	
	// Determine which order the quadrature must be sent by determining if the
	// new velocity is greater or less than the previously commanded velocity
	// If greater, Input A begins the quadrature. If less, Input B begins the
	// quadrature.
	int32_t curr_velocity_rounded = round(current_velocity / velocity_resolution);
	int32_t target_velocity_rounded = round(target_velocity / velocity_resolution);
	int32_t velocity_difference = labs(target_velocity_rounded - curr_velocity_rounded);
	
	// If no difference in current vs. target velocity, exit function
	if (velocity_difference == 0) {
		return;
	}
	
	for (int32_t i = 0; i < velocity_difference; ++i) {
		// If a flag is raised via interrupts
		if (e_stop_flag || neg_lim_switch_flag || pos_lim_switch_flag) {
			target_velocity = 0;
		}
		if (target_velocity > current_velocity) {
			// Toggle Input A to begin the quadrature signal
			motor.MotorInAState(true);
			// Command a 5 microsecond delay to ensure proper signal timing
			Delay_us(5);
			motor.MotorInBState(true);
			Delay_us(5);
			motor.MotorInAState(false);
			Delay_us(5);
			motor.MotorInBState(false);
			Delay_us(5);
		}
		else {
			motor.MotorInBState(true);
			Delay_us(5);
			motor.MotorInAState(true);
			Delay_us(5);
			motor.MotorInBState(false);
			Delay_us(5);
			motor.MotorInAState(false);
			Delay_us(5);
		}
	}
	
	// Update the current velocity
	current_velocity = target_velocity;
		
	// Wait for High-Level Feedback (HLFB) to assert (signaling if the motor has reached
	// its target velocity)
#if __SERIAL_DEBUG__
	ConnectorUsb.SendLine("Ramping speed, waiting for HLFB.");
#endif

	//assert_HLFB();    // Comment out to improve function speed
		
	// Check to see if motor faulted during move
	if (check_for_faults()) {
#if __SERIAL_DEBUG__
		ConnectorUsb.SendLine("Motion may not have completed as expected. Proceed with caution.");
	}
	else {
		ConnectorUsb.SendLine("Move done.");
#endif
	}

}

void ClearPathMC::calibrate() {
	/* Send the moving base to the motor-side (negative) limit switch */
	while (system_status == slidersystem::SYSTEM_CALIBRATING) {
		// TODO: receive message telling the calibration to be performed on the neg or pos limit switch		
		if (neg_lim_switch_flag) {
			set_velocity(0, &system_status);
			move_at_target_velocity();
			neg_lim_switch_flag = false;
			system_status = slidersystem::NEG_LIM;
			return;
 		}
		
		// Exit calibration if E-stop
		if (e_stop_flag) {
			system_status = slidersystem::E_STOP;
			return;
		}

		// During calibration, move toward negative limit switch
		set_velocity(m_calibration_velocity, &system_status);
		move_at_target_velocity();
		//_eth.send_packet(&system_status, target_velocity);
	}
}