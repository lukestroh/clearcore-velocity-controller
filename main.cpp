/*
	* Author: Luke Strohbehn
	* Created: 09/21/2023
	* 
	* Requirements:
	*
	* ETHERNET COMMUNICATION:
	* 1. The PC should be running software capable of sending and receiving UDP 
	*    packets. See `udp_client.py` for simple testing.
	*
	*
	* MOTOR:
	* 1. A ClearPath motor must be connected to Connector M-0.
	* 2. The connected ClearPath motor must be configured through the MSP software
	*    for Manual Velocity Control mode (In MSP select Mode>>Velocity>>Manual
	*    Velocity Control, then hit the OK button).
	* 3. In the MSP software:
	*    * Define a Max Clockwise and Counter-Clockwise (CW/CCW) Velocity (On the
	*      main MSP window fill in the textboxes labeled "Max CW Velocity (RPM)"
	*      and "Max CCW Velocity (RPM)"). Any velocity commanded outside of this
	*      range will be rejected.
	*    * Set the Velocity Resolution to 2 (On the main MSP window check the
	*      textbox labeled "Velocity Resolution (RPM per knob count)" 2 is
	*      default). This means the commanded velocity will always be a multiple
	*      of 2. For finer resolution, lower this value and change
	*      velocityResolution in the sketch below to match.
	*    * Set Knob Direction to As-Wired, and check the Has Detents box (On the
	*      main MSP window check the dropdown labeled "Knob Direction" and the
	*      checkbox directly below it labeled "Has Detents").
	*    * On the main MSP window set the dropdown labeled "On Enable..." to be
	*      "Zero Velocity".
	*    * Set the HLFB mode to "ASG-Velocity w/Measured Torque" with a PWM carrier
	*      frequency of 482 Hz through the MSP software (select Advanced>>High
	*      Level Feedback [Mode]... then choose "ASG-Velocity w/Measured Torque"
	*      from the dropdown, make sure that 482 Hz is selected in the "PWM Carrier
	*      Frequency" dropdown, and hit the OK button).
	*
	* LIMIT SWITCHES:
	* 1. Limit switches should be connected to the DI8 and DI7 inputs on the controller.
	*
	*
	* EMERGENCY STOP:
	* 1. Emergency stop should be connected to the DI-6 input on the controller.
	*
	* Links:
	* ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
	* ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
	*
	*
	* Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
	* the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

#ifndef __SERIAL_DEBUG__
#define __SERIAL_DEBUG__ 1
#endif

#include "ClearCore.h"
#include "EthUDP.h"
#include "ClearPathMC.h"
#include "system.h"

#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__ || __CPMC_DEBUG__
#define __SET_UP_SERIAL__ 1
#else
#define __SET_UP_SERIAL__ 0
#endif


// System state variables
volatile bool neg_lim_switch_flag = false;
volatile bool pos_lim_switch_flag = false;
volatile bool e_stop_flag = false;
constexpr uint8_t DEBOUNCE_TIME = 10;



#if __SERIAL_DEBUG__ || __SET_UP_SERIAL__
void set_up_serial(void) {
	/* Set up Serial communication with computer for debugging */
	ConnectorUsb.Mode(Connector::USB_CDC);
	ConnectorUsb.Speed(115200);
	ConnectorUsb.PortOpen();
	uint32_t serial_timeout = 5000;
	uint32_t start_time = Milliseconds();
	while (!ConnectorUsb && Milliseconds() - start_time < serial_timeout) {
		continue;
	}
}
#endif // __SERIAL_DEBUG__

bool read_switch(DigitalIn* switch_pin, volatile bool* p_interrupt_flag) {
	/* Returns true if the interrupt has been triggered */
	if (*p_interrupt_flag) {
		bool reading = switch_pin->State();
		static unsigned long prev_time;
		static bool change_pending = false;
		if (reading) {
			change_pending = true;
		}
		if (!reading && change_pending) {
			if (Milliseconds() - prev_time > DEBOUNCE_TIME) {
				*p_interrupt_flag = false;
				change_pending = false;
				return true;
			}
		}
	}	
	return false;
}

bool poll_switch(DigitalIn* switch_pin) {
	return switch_pin->State();
}


int main(void) {
#if __SET_UP_SERIAL__
	set_up_serial();
#endif

	// Set static address for the ClearCore Controller
	IpAddress local_ip(169, 254, 57, 177);
	// Set remote (host) computer address
	IpAddress remote_ip(169, 254, 57, 209);

	EthUDP eth(local_ip, 8888, remote_ip, 44644);

	ClearPathMC motor0(0);

	eth.begin();
	motor0.begin();
	if (poll_switch(&motor0.emergency_stop_pin)) {
		motor0.state_.system_status = slidersystem::E_STOP;
		e_stop_flag = true;
	}
	eth.send_packet(&motor0.state_);
		
	// Main loop
	while (true) {
		// Read data from the ROS2 hardware interface, store in motor command interface.
		eth.read_packet(&motor0.command_);
				
		// If new data, parse for new motor control
		if (eth.new_data) {
			
			#if __SERIAL_DEBUG__
			//ConnectorUsb.Send("Command - status: ");
			//ConnectorUsb.Send(motor0.command_.system_status);
			//ConnectorUsb.Send(" rpm: ");
			//ConnectorUsb.SendLine(motor0.command_.vel);
			//
			//ConnectorUsb.Send("State - status: ");
			//ConnectorUsb.Send(motor0.state_.system_status);
			//ConnectorUsb.Send(" rpm: ");
			//ConnectorUsb.SendLine(motor0.state_.vel);
			#endif
			switch (motor0.command_.system_status) {
				case slidersystem::E_STOP:
					e_stop_flag = true;
					break;
				case slidersystem::SYSTEM_OK: // set_velocity() deals with case, but with state_. Is it worth doing an additional check here? Looks like it's faster...
					// Set the new target velocity
					ConnectorUsb.SendLine("YES");
					motor0.state_.system_status = slidersystem::SYSTEM_OK; // TODO: does this defeat the point of having a standby check in the set_velocity function?
					if (motor0.command_.vel > motor0.state_.vel) {
						motor0.set_velocity(motor0.state_.vel + 1);
					}
					else if (motor0.command_.vel < motor0.state_.vel) {
						motor0.set_velocity(motor0.state_.vel - 1);
					}
					break;
				case slidersystem::SYSTEM_STANDBY:
					motor0.state_.system_status = slidersystem::SYSTEM_STANDBY;
					motor0.set_standby();
					break;
				case slidersystem::SYSTEM_CALIBRATING:
					// Poll the pin to see if the slider is already at the switch. // TODO: get emergency-emergency limit switches?
					if (!poll_switch(&motor0.limit_switch_pin_neg)) {
						motor0.state_.system_status = slidersystem::NEG_LIM;
					}
					else {
						motor0.state_.system_status = slidersystem::SYSTEM_CALIBRATING;
						eth.send_packet(&motor0.state_);
						motor0.calibrate(); // blocking, runs until negative limit switch hit. TODO: change to either side
					}
					break;
				// check the limit switch statuses TODO: does this even need to be checked? Does host ever command a switch? NO.
				case slidersystem::NEG_LIM:
					if (motor0.state_.vel <= 0){
						#if __SERIAL_DEBUG__
						ConnectorUsb.SendLine("Commanded velocity was stopped by the negative limit switch");
						#endif
						motor0.set_velocity(0);
					}
					break;
				case slidersystem::POS_LIM:
					if (motor0.state_.vel >= 0){
						#if __SERIAL_DEBUG__
						ConnectorUsb.SendLine("Commanded velocity was stopped by the positive limit switch");
						#endif
						motor0.set_velocity(0);
					}
					break;
			}
			eth.new_data = false;
		}
		
		// Limit switch check
		if (neg_lim_switch_flag) {
			motor0.set_velocity(0);
			motor0.move_at_target_velocity();
			motor0.state_.system_status = slidersystem::NEG_LIM;
			neg_lim_switch_flag = false;
		}
		if (pos_lim_switch_flag) {
			motor0.set_velocity(0);
			motor0.move_at_target_velocity();
			motor0.state_.system_status = slidersystem::POS_LIM;
			pos_lim_switch_flag = false;
		}

		// E stop check
		if (e_stop_flag) {
			while (1) {
				motor0.set_velocity(0);
				motor0.move_at_target_velocity();
				motor0.state_.system_status = slidersystem::E_STOP;
#if __SERIAL_DEBUG__
				ConnectorUsb.SendLine("EMERGENCY STOP TRIGGERED. CHECK ALL HARDWARE.");
#endif
				eth.send_packet(&motor0.state_);
				//return 255;
				Delay_ms(5000);
			}
		}
		
		if (!poll_switch(&motor0.limit_switch_pin_neg)) {        // This is now working as it should?? Can it be?
			motor0.state_.system_status = slidersystem::NEG_LIM;
		}
		if (!poll_switch(&motor0.limit_switch_pin_pos)) {
			motor0.state_.system_status = slidersystem::POS_LIM;
		}
		
		// Poll E-stop so user knows to properly reset the switch TODO: move all e-stop stuff to interrupt
		if (poll_switch(&motor0.emergency_stop_pin)) {
			motor0.state_.system_status = slidersystem::E_STOP;
			e_stop_flag = true;
		}
		
		// Move to target velocity (blocking)
		motor0.move_at_target_velocity();
		
		// Send status, velocity data to the ROS2 node.
		eth.send_packet(&motor0.state_);
	}
}