/*
 * EthUDP.cpp
 *
 * Created: 9/21/2023 6:20:23 PM
 * Author: Luke Strohbehn
 */ 

#ifndef __SERIAL_DEBUG__
#define __SERIAL_DEBUG__ 0
#endif

#include "EthUDP.h"

EthUDP::EthUDP():
	/* Initialize class variables */
	m_local_ip(169, 254, 97, 177),
	m_local_port {8888},
	m_remote_ip(169, 254, 57, 209),
	m_remote_port {8888}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip):
	m_local_ip(_local_ip),
	m_local_port {8888},
	m_remote_ip(169, 254, 57, 209),
	m_remote_port {8888}
{
	
	
}

EthUDP::EthUDP(IpAddress _local_ip, int _local_port):
	m_local_ip(_local_ip),
	m_local_port(_local_port),
	m_remote_ip(169, 254, 57, 209),
	m_remote_port {8888}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip, IpAddress _remote_ip):
	m_local_ip(_local_ip),
	m_local_port {8888},
	m_remote_ip(_remote_ip),
	m_remote_port {8888}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip, int _local_port, IpAddress _remote_ip, int _remote_port):
	m_local_ip(_local_ip),
	m_local_port(_local_port),
	m_remote_ip(_remote_ip),
	m_remote_port(_remote_port)
{

}

EthUDP::~EthUDP() {}

void EthUDP::begin(void) {
	/* Set up UDP Ethernet communication */
	// Check physical Ethernet link

	while (!EthernetMgr.PhyLinkActive()) {
#if __SERIAL_DEBUG__
		ConnectorUsb.SendLine("Could not detect a physical Ethernet connection.");
#endif
		Delay_ms(1000);
	}
	
	// Run the setup for the ClearCore Ethernet manager
	EthernetMgr.Setup();
	if (m_using_dhcp) {
		bool dhcp_success = EthernetMgr.DhcpBegin();
		if (dhcp_success) {
#if __SERIAL_DEBUG__
			ConnectorUsb.Send("DHCP successfully assigned an IP address: ");
			ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
#endif
		}
		else {
			while (true) {
#if __SERIAL_DEBUG__
				ConnectorUsb.SendLine("DHCP configuration was unsuccessful.");
#endif
				Delay_ms(10000);
			}
		}
	}
	else {
		EthernetMgr.LocalIp(m_local_ip);
		//EthernetMgr.GatewayIp(IpAddress(169,254, 93, 234));
		//EthernetMgr.NetmaskIp(IpAddress(255, 255, 0, 0));
	}
	
	// Begin listening on the local port for UDP datagrams
	udp.Begin(m_local_port);
}


void EthUDP::read_packet(slidersystem::DataInterface* command_interface) {
	/* Look for a received packet, store in 'received_packet' if present */
	uint16_t packet_size = udp.PacketParse();
	if (packet_size > 0) {
		udp.PacketRead(received_packet, MAX_PACKET_LENGTH);
		new_data = true;
		
		// Parse data from the received packet
		// Extract first field
		char* received_packet_cstr = reinterpret_cast<char*>(received_packet);
		m_token = strtok(received_packet_cstr, m_delimiter);
		
		if (m_token != NULL) {
					//#ifdef __SERIAL_DEBUG__
					//ConnectorUsb.SendLine(m_token);
					//#endif
			command_interface->system_status = static_cast<slidersystem::SystemStatus>(atoi(m_token));			
			// Extract second field
			m_token = strtok(NULL, m_delimiter);
			//token_cstr = reinterpret_cast<char*>(token);
			if (m_token != NULL) {
				//#ifdef __SERIAL_DEBUG__
				//ConnectorUsb.SendLine("got far");
				//ConnectorUsb.SendLine(m_token);
				//#endif
				command_interface->vel = atof(m_token);
			}
		}		
	}
}

void EthUDP::construct_data_msg(slidersystem::DataInterface* state) {
	/* Construct the message to send to the ROS2 Node on the host computer 
	https://stackoverflow.com/questions/23966080/sending-struct-over-udp-c
	*/
	// Reset buffers
	memset(&msg_buf[0], 0, sizeof(msg_buf));
	memset(&status_buf[0], 0, sizeof(status_buf));
	memset(&data_buf[0], 0, sizeof(data_buf));
	
	// Set data
	sprintf(status_buf, "%d", state->system_status);
	sprintf(data_buf, "%f", state->vel * -1); // x direction flipped in ros2 --> TODO: move all of the negative signs into one place! This shouldn't be here.
	
	// Create c-str msg
	strcat(msg_buf, status_header); // TODO: For some reason status_header gets set to 0. Needs a debugger.
	strcat(msg_buf, status_buf);
	strcat(msg_buf, m_delimiter);
	strcat(msg_buf, data_header);
	strcat(msg_buf, data_buf);
	strcat(msg_buf, footer);
	
	#ifdef __SERIAL_DEBUG__
	ConnectorUsb.SendLine(msg_buf);
	//ConnectorUsb.SendLine(motor0.command_.system_status);
	//ConnectorUsb.SendLine(motor0.command_.vel);
	//ConnectorUsb.SendLine(motor0.state_.vel);
	#endif
}


void EthUDP::send_packet(slidersystem::DataInterface* state) {
	/* Send a packet */
	construct_data_msg(state);
	udp.Connect(m_remote_ip, m_remote_port);
	udp.PacketWrite(msg_buf);
	udp.PacketSend();
}