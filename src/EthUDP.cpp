/*
 * EthUDP.cpp
 *
 * Created: 9/21/2023 6:20:23 PM
 * Author: Luke Strohbehn
 */ 


#include "EthUDP.h"

EthUDP::EthUDP():
	/* Initialize class variables */
	local_ip(169, 254, 97, 177),
	local_port {8888},
	remote_ip(169, 254, 57, 209),
	remote_port {8888}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip):
	local_ip(_local_ip),
	local_port {8888},
	remote_ip(169, 254, 57, 209),
	remote_port {8888}
{
	
	
}

EthUDP::EthUDP(IpAddress _local_ip, int _local_port):
	local_ip(_local_ip),
	local_port(_local_port),
	remote_ip(169, 254, 57, 209),
	remote_port {8888}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip, IpAddress _remote_ip):
	local_ip(_local_ip),
	local_port {8888},
	remote_ip(_remote_ip),
	remote_port {8888}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip, int _local_port, IpAddress _remote_ip, int _remote_port):
	local_ip(_local_ip),
	local_port(_local_port),
	remote_ip(_remote_ip),
	remote_port(_remote_port)
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
		//EthernetMgr.GatewayIp(IpAddress(169,254, 93, 234));
		//EthernetMgr.NetmaskIp(IpAddress(255, 255, 0, 0));
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

char* EthUDP::construct_data_msg(slidersystem::SystemStatus system_status, float data) {
	/* Construct the message to send to the ROS2 Node on the host computer 
	
	https://stackoverflow.com/questions/23966080/sending-struct-over-udp-c
	*/
	
	memset(&msg_buf[0], 0, sizeof(msg_buf));
	char status_buf[2];
	sprintf(status_buf, "%d", system_status);
	char data_buf[10];
	sprintf(data_buf, "%f", data);
	char status_header[11] = "{\"status\":";
	char vel_header[13] = "\"servo_rpm\":";
	char footer[2] = "}";
	
	strcpy(msg_buf, status_header);
	strcat(msg_buf, status_buf);
	strcat(msg_buf, ",");
	strcat(msg_buf, vel_header);
	strcat(msg_buf, data_buf);
	strcat(msg_buf, footer);
	return msg_buf;
}


void EthUDP::send_packet(slidersystem::SystemStatus system_status, float data) {
	/* Send a packet */
	char* msg = construct_data_msg(system_status, data);
	
	udp.Connect(remote_ip, remote_port);
	udp.PacketWrite(msg);
	udp.PacketSend();
}


