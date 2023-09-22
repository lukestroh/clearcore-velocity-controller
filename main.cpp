/*
 * Author: Luke Strohbehn
 * Created: 09/21/2023
 * 
 * Requirements:
 * ** Setup 1 (ClearCore and a PC): The PC should be running software capable
 *    of sending and receiving UDP packets. PacketSender is highly recommended
 *    as a free, cross-platform software. Configure PacketSender to send a UDP
 *    packet to the ClearCore by specifying the IP address and port provided to
 *    EthernetMgr.LocalIp(). Your firewall or network settings may need to be
 *    adjusted in order to receive the response back from the ClearCore.
 * ** Setup 2 (ClearCore to a ClearCore): A partner sketch is included at the
 *    end of this file that can be used on the other ClearCore. The MAC address
 *    and IP address values set up for each ClearCore must be unique. The remote
 *    IP address and port used in the partner sketch must match the IP address
 *    and port used to setup the ClearCore in this sketch.
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
#include "EthernetUdp.h"
#include "EthUDP.h"

//// Set a static address for the ClearCore Controller
//IpAddress ip = IpAddress(169, 254, 97, 177);

EthUDP eth;

// Target velocity
float target_velocity {0.0};



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
	
	while (true) {
		eth.read_packet();
		
		if (eth.new_data) {
			ConnectorUsb.Send("Data received from address ");
			ConnectorUsb.Send(eth.udp.RemoteIp().StringValue());
			ConnectorUsb.Send(": ");
			target_velocity = atof(reinterpret_cast<const char*>(eth.received_packet));
			ConnectorUsb.SendLine(target_velocity);
			eth.new_data = false;
		}
		
		eth.send_packet(2.712);
	}
}