/*
 * ClearPathMC.cpp
 *
 * Created: 9/25/2023 11:37:01 AM
 * Author: Luke Strohbehn
 */

#include "ClearPathMC.h"

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
	ConnectorUsb.SendLine("Motor enabled.");
		
	// Wait for HLFB
	assert_HLFB();
	
	ConnectorUsb.SendLine("Motor setup complete.");
}


bool ClearPathMC::check_for_faults() {
	/* Check if a motor fault is currently preventing motion, clear fault if configured to do so.
	 * Returns true if in fault.
	*/
	if (motor.StatusReg().bit.MotorInFault) {
		if (HANDLE_MOTOR_FAULTS) {
			ConnectorUsb.SendLine("Motor fault detected. Move canceled.");
			handle_motor_faults();
		}
		else {
			ConnectorUsb.SendLine("Motor fault detected. Move canceled. Enable automatic fault handling by setting HANDLE_MOTOR FAULTS to 1.");
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
 	ConnectorUsb.SendLine("Handling fault: clearing faults by cycling enable signal to motor.");
	motor.EnableRequest(false);
	Delay_ms(10);
	motor.EnableRequest(true);
	Delay_ms(100);
 }
 

void ClearPathMC::assert_HLFB() {
	/* Make sure the HLFB is connected */
	while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED && !motor.StatusReg().bit.MotorInFault) {
		ConnectorUsb.SendLine("ERROR IN HLFB ASSERT:");
		ConnectorUsb.Send("\tHLFB STATE: ");
		ConnectorUsb.SendLine(motor.HlfbState());
		ConnectorUsb.Send("\tMOTOR IN FAULT: ");
		ConnectorUsb.SendLine(motor.StatusReg().bit.MotorInFault);
		
		ConnectorUsb.Send("\tHLFB Percent: ");
		ConnectorUsb.SendLine(motor.HlfbPercent());
		Delay_ms(100);
		continue;
	}
}


void ClearPathMC::get_velocity() {
	
}

void ClearPathMC::set_velocity(double vel) {
	/* Set the target velocity of the ClearPath MC motor, according to maximum velocity limits */
	if (vel > max_velocity_CCW) {
		target_velocity = max_velocity_CW;
	}
	else if (vel < -max_velocity_CCW) {
		target_velocity = max_velocity_CCW;
	}
	else {
		target_velocity = vel;
	}
}


void ClearPathMC::move_at_target_velocity() {
	/* Move the motor at the set target velocity */
	
	// Check motor status
	check_for_faults();
	
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
	ConnectorUsb.SendLine("Ramping speed, waiting for HLFB.");
	assert_HLFB();
		
	// Check to see if motor faulted during move
	if (check_for_faults()) {
		ConnectorUsb.SendLine("Motion may not have completed as expected. Proceed with caution.");
	}
	else {
		ConnectorUsb.SendLine("Move done.");
	}	
}