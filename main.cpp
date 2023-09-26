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

//// Set a static address for the ClearCore Controller
//IpAddress ip = IpAddress(169, 254, 97, 177);

EthUDP eth;

ClearPathMC motor0;


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




int main(void) {
	set_up_serial();
	eth.begin();
	motor0.begin();
	
	while (true) {
		// Read data from the ROS2 node.
		eth.read_packet();
		
		// If new data, parse for new motor control
		if (eth.new_data) {
			ConnectorUsb.Send("Data received from address ");
			ConnectorUsb.Send(eth.udp.RemoteIp().StringValue());
			ConnectorUsb.Send(": ");
			motor0.set_velocity(atof(reinterpret_cast<const char*>(eth.received_packet)));
			ConnectorUsb.SendLine(motor0.target_velocity);
			eth.new_data = false;
		}
		
		// Send data to the ROS2 node.
		eth.send_packet(2.712);
	}
}