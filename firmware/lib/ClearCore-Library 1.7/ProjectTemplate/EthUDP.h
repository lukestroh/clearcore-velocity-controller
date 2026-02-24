/*
 * EthUDP.h
 *
 * Created: 9/21/2023 6:16:53 PM
 * Author: Luke Strohbehn
 */ 
#include "ClearCore.h"
#include "EthernetUdp.h"
#include "system.h"

#ifndef ETHUDP_H_
#define ETHUDP_H_

#ifndef __ETHUDP_DEBUG__
#define __ETHUDP_DEBUG__ 0
#endif

class EthUDP {
	private:
		// Local IP address, port
		IpAddress m_local_ip;
		const int m_local_port;
		
		// Host IP address, port
		IpAddress m_remote_ip;
		const int m_remote_port;
		
		// Read data attributes
		char* m_token;
		const char* m_delimiter = ",";
		bool m_using_dhcp = false;
		const uint8_t MAX_PACKET_LENGTH = 128; // Maximum number of characters to receive from an incoming packet
		
		// Send data attributes
		char m_status_buf[2];
		char m_data_buf[10];
		const char* m_msg_status_header = "{\"status\":";
		const char* m_msg_data_header = "\"servo_rpm\":";
		const char* m_msg_footer = "}";		
		
		// Data buffers
		unsigned char m_received_packet[128]; // Buffer for holding received packets
		char m_msg_buf[128]; // Send message buffer	
	
	public:
		// New data flag
		bool new_data = false;

		
		// Ethernet UDP
		EthernetUdp udp;
		
		EthUDP();
		EthUDP(IpAddress _local_ip);
		EthUDP(IpAddress _local_ip, int _local_port);
		EthUDP(IpAddress _local_ip, IpAddress _remote_ip);
		EthUDP(IpAddress _local_ip, int _local_port, IpAddress _remote_ip, int _remote_port);
		~EthUDP();
	
		// Public methods
		void begin();
		void read_packet(slidersystem::DataInterface* command_interface);
		void construct_data_msg(slidersystem::DataInterface* state);
		void send_packet(slidersystem::DataInterface* state);
		
};

#endif /* ETHUDP_H_ */