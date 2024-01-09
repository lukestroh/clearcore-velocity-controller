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
	* 1. Limit switches should be connected to the I0 and I1 inputs on the controller.
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


#include "ClearCore.h"
#include "EthUDP.h"
#include "ClearPathMC.h"
#include "system.h"

volatile bool neg_lim_switch_flag = false;
volatile bool pos_lim_switch_flag = false;
volatile bool e_stop_flag = false;

// Set static addresses for the ClearCore Controller
IpAddress local_ip = IpAddress(169, 254, 97, 177);
IpAddress remote_ip = IpAddress(169, 254, 57, 209);

EthUDP eth(local_ip, remote_ip);

ClearPathMC motor0(0);

// System state variable
volatile slidersystem::SystemStatus system_state = slidersystem::SYSTEM_STANDBY;

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

void reset_slider(void) {
	/* Reset the linear slider to position 0, where base is closest to the motor. 
	   Run the motor at a low constant velocity until the negative limit switch is trigged
	   and send a message via Ethernet to ROS2. */ 
	motor0.target_velocity = -30;
	while (!neg_lim_switch_flag) {
		motor0.move_at_target_velocity();
		eth.send_packet(system_state, motor0.target_velocity);
	}
	motor0.target_velocity = 0;
	motor0.move_at_target_velocity(true);
}

bool read_switch(DigitalIn& switch_pin, bool& interrupt_flag) {
	if (interrupt_flag) {
		bool reading = switch_pin.State();
		static bool change_pending = false;
		if (!reading) {
			change_pending = true;
		}
		if (reading && change_pending) {
			interrupt_flag = false;
			change_pending = false;
			return true;
		}
	}
	return false;
}


int main(void) {
	set_up_serial();
	eth.begin();
	motor0.begin();
	
	reset_slider();
	
	while (true) {
		// Read data from the ROS2 hardware interface.
		eth.read_packet();
		
		// If new data, parse for new motor control
		if (eth.new_data) {
			// Set the new target velocity
			motor0.set_velocity(atof(reinterpret_cast<const char*>(eth.received_packet)));
			eth.new_data = false;
		}
		
		// Limit switch check
		if (neg_lim_switch_flag) {
			motor0.target_velocity = 0;
			motor0.move_at_target_velocity(true);
			//system_state = slidersystem::SystemStatus::NEG_LIM;
		}
		if (pos_lim_switch_flag) {
			motor0.target_velocity = 0;
			motor0.move_at_target_velocity(true);
			//system_state = slidersystem::SystemStatus::POS_LIM;
		}
		
		// E stop check
		while (e_stop_flag) {
			motor0.target_velocity = 0;
			motor0.move_at_target_velocity(true);
			//system_state = slidersystem::SystemStatus::E_STOP;
			ConnectorUsb.SendLine("EMERGENCY STOP TRIGGERED. CHECK ALL HARDWARE.");
			eth.send_packet(system_state, motor0.target_velocity);
			Delay_ms(1000);
		}
		
		// Move to target velocity (blocking)
		motor0.move_at_target_velocity();
		
		//// Get current velocity
		//float motor_vel = motor0.get_velocity();
		//ConnectorUsb.SendLine(motor_vel);
		ConnectorUsb.SendLine(motor0.target_velocity);
		
		// Send status, velocity data to the ROS2 node.
		eth.send_packet(system_state, motor0.target_velocity);		
	}
}