/*
 * EthUDP.h
 *
 * Created: 9/21/2023 6:16:53 PM
 * Author: Luke Strohbehn
 */ 
#include "ClearCore.h"
#include "EthernetUdp.h"

#ifndef ETHUDP_H_
#define ETHUDP_H_

class EthUDP {
	private:
		// Local IP address, port
		IpAddress local_ip;
		const int local_port;
		
		// Host IP address, port
		IpAddress remote_ip = IpAddress(169, 254, 93, 234);
		const int remote_port = 8888;
	
	public:
		// Data buffer
		bool new_data = false;
		bool using_dhcp = false;
		const uint8_t MAX_PACKET_LENGTH = 128; // Maximum number of characters to receive from an incoming packet
		unsigned char received_packet[128]; // Buffer for holding received packets
		char msg_buf[128];
		
		// Ethernet UDP
		EthernetUdp udp;
		
		EthUDP();
		EthUDP(IpAddress _ip);
		EthUDP(IpAddress _ip, int _port);
		~EthUDP();		
		
		// Public methods
		void begin();
		void read_packet();
		char* construct_data_msg(float data); // // Set true if using DHCP to configure the local IP address
		void send_packet(float data);
};

#endif /* ETHUDP_H_ */