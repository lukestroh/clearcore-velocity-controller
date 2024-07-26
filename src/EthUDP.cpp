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
	m_local_ip(169, 254, 57, 177),
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
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
		ConnectorUsb.SendLine("Could not detect a physical Ethernet connection.");
#endif
		Delay_ms(1000);
	}
	
	// Run the setup for the ClearCore Ethernet manager
	EthernetMgr.Setup();
	if (m_using_dhcp) {
		bool dhcp_success = EthernetMgr.DhcpBegin();
		if (dhcp_success) {
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
			ConnectorUsb.Send("DHCP successfully assigned an IP address: ");
			ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
#endif
		}
		else {
			while (true) {
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
				ConnectorUsb.SendLine("DHCP configuration was unsuccessful.");
#endif
				Delay_ms(10000);
			}
		}
	}
	else {
		EthernetMgr.LocalIp(m_local_ip);
		//EthernetMgr.GatewayIp(IpAddress(169,254, 93, 234)); // TODO: add these to the constructors
		EthernetMgr.NetmaskIp(IpAddress(255, 255, 0, 0));
	}
	
	// Begin listening on the local port for UDP datagrams
	udp.Begin(m_local_port);
}


void EthUDP::read_packet(slidersystem::DataInterface* command_interface) {
	/* Look for a received packet, store in 'received_packet' if present */
	uint16_t packet_size = udp.PacketParse();
	if (packet_size > 0) {
		udp.PacketRead(m_received_packet, MAX_PACKET_LENGTH);
		new_data = true;
				
		// Parse data from the received packet
		// Extract first field
		char* received_packet_cstr = reinterpret_cast<char*>(m_received_packet);
		m_token = strtok(received_packet_cstr, m_delimiter);

		
		if (m_token != NULL) {
			command_interface->system_status = static_cast<slidersystem::SystemStatus>(atoi(m_token));			
			// Extract second field
			m_token = strtok(NULL, m_delimiter);
			//token_cstr = reinterpret_cast<char*>(token);
			if (m_token != NULL) {
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
	memset(&m_msg_buf[0], 0, sizeof(m_msg_buf));
	memset(&m_status_buf[0], 0, sizeof(m_status_buf));
	memset(&m_data_buf[0], 0, sizeof(m_data_buf));
	
	// Set data
	sprintf(m_status_buf, "%d", state->system_status);
	sprintf(m_data_buf, "%f", state->vel);
	
	// Create c-str msg
	strcat(m_msg_buf, m_msg_status_header); // TODO: For some reason status_header gets set to 0. Needs a debugger.
	strcat(m_msg_buf, m_status_buf);
	strcat(m_msg_buf, m_delimiter);
	strcat(m_msg_buf, m_msg_data_header);
	strcat(m_msg_buf, m_data_buf);
	strcat(m_msg_buf, m_msg_footer);
	
#if __SERIAL_DEBUG__ // || __ETHUDP_DEBUG__
	ConnectorUsb.Send("Constructed msg: ");
	ConnectorUsb.SendLine(m_msg_buf);
#endif
}


void EthUDP::send_packet(slidersystem::DataInterface* state) {
	/* Send a packet */
	construct_data_msg(state);
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
	ConnectorUsb.Send("Sending msg: ");
	ConnectorUsb.SendLine(m_msg_buf);
#endif
	udp.Connect(m_remote_ip, m_remote_port);
	udp.PacketWrite(m_msg_buf);
	udp.PacketSend();
} 