/*
 * EthUDP.cpp
 *
 * Created: 9/21/2023 6:20:23 PM
 *  Author: Luke Strohbehn
 */ 


#include "EthUDP.h"

EthUDP::EthUDP():
	/* Initialize class variables */
	local_ip(169, 254, 97, 177),
	local_port {8888}
{
		
}

EthUDP::~EthUDP() {}

void EthUDP::begin(void) {
	/* Set up UDP Ethernet communication */
	// Check physical Ethernet link
	while (!EthernetMgr.PhyLinkActive()) {
		ConnectorUsb.SendLine("Could not detect a physical Ethernet connection.");
		Delay_ms(1000);
	}
	
	// Run the setup for the ClearCore Ethernet manager
	EthernetMgr.Setup();
	if (using_dhcp) {
		bool dhcp_success = EthernetMgr.DhcpBegin();
		if (dhcp_success) {
			ConnectorUsb.Send("DHCP successfully assigned an IP address: ");
			ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
		}
		else {
			while (1) {
				ConnectorUsb.SendLine("DHCP configuration was unsuccessful.");
				Delay_ms(10000);
				continue;
			}
		}
	}
	else {
		EthernetMgr.LocalIp(local_ip);
	}
	
	// Begin listening on the local port for UDP datagrams
	udp.Begin(local_port);
}


void EthUDP::read_packet(void) {
	/* Look for a received packet, store in 'received_packet' if present */
	uint16_t packet_size = udp.PacketParse();
	if (packet_size > 0) {
		uint32_t bytes_read = udp.PacketRead(received_packet, MAX_PACKET_LENGTH);
		new_data = true;
	}
}

char* EthUDP::construct_data_msg(float data) {
	/* Construct the message to send to the ROS2 Node on the host computer */
	memset(&msg_buf[0], 0, sizeof(msg_buf));
	char data_buf[10];
	sprintf(data_buf, "%f", data);
	char header[19] = "{'servo_velocity':";
	char footer[2] = "}";
	
	strcpy(msg_buf, header);
	strcat(msg_buf, data_buf);
	strcat(msg_buf, footer);
	return msg_buf;
}


void EthUDP::send_packet(float data) {
	/* Send a packet */
	char* msg = construct_data_msg(data);
	
	udp.Connect(udp.RemoteIp(), udp.RemotePort());
	udp.PacketWrite(msg);
	udp.PacketSend();
}

