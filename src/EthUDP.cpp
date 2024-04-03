/*
 * EthUDP.cpp
 *
 * Created: 9/21/2023 6:20:23 PM
 * Author: Luke Strohbehn
 */ 

#ifndef __SERIAL_DEBUG__
#define __SERIAL_DEBUG__ 1
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


void EthUDP::read_packet(void) {
	/* Look for a received packet, store in 'received_packet' if present */
	uint16_t packet_size = udp.PacketParse();
	if (packet_size > 0) {
		uint32_t bytes_read = udp.PacketRead(received_packet, MAX_PACKET_LENGTH);
		new_data = true;
		
		// Parse data from the received packet
		// Extract first field
		char* received_packet_cstr = reinterpret_cast<char*>(received_packet);
		token = strtok(received_packet_cstr, delimiter);
		
		if (token != NULL) {
			command_data.status = static_cast<int8_t>(atoi(token));
//#if __SERIAL_DEBUG__
//ConnectorUsb.Send("Field 1 is:");
//ConnectorUsb.SendLine(command_data.status);
//#endif
			
			// Extract second field
			token = strtok(NULL, delimiter);
			//token_cstr = reinterpret_cast<char*>(token);
			if (token != NULL) {
				command_data.vel_command = atof(token);
//#if __SERIAL_DEBUG__
//ConnectorUsb.Send("Field 2 is:");
//ConnectorUsb.SendLine(command_data.vel_command);
//#endif
			}
		}		
	}
}

void EthUDP::construct_data_msg(slidersystem::SystemStatus system_status, float data) {
	/* Construct the message to send to the ROS2 Node on the host computer 
	https://stackoverflow.com/questions/23966080/sending-struct-over-udp-c
	*/
	// Reset buffers
	ConnectorUsb.SendLine(status_header);
	memset(&msg_buf[0], 0, sizeof(msg_buf));
	memset(&status_buf[0], 0, sizeof(status_buf));
	memset(&data_buf[0], 0, sizeof(data_buf));
	
	// Set data
	sprintf(status_buf, "%d", system_status);
	sprintf(data_buf, "%f", data);
	
	// Create c-str msg
	strcpy(msg_buf, status_header);
	strcat(msg_buf, status_buf);
	strcat(msg_buf, ",");
	strcat(msg_buf, data_header);
	strcat(msg_buf, data_buf);
	strcat(msg_buf, footer);
}


void EthUDP::send_packet(slidersystem::SystemStatus system_status, float data) {
	/* Send a packet */
	construct_data_msg(system_status, data);
	udp.Connect(m_remote_ip, m_remote_port);
	udp.PacketWrite(msg_buf);
	udp.PacketSend();
}