/*
 * system.h
 *
 * Created: 1/5/2024 10:07:56 PM
 *  Author: Luke Strohbehn
 * 
 * 
 *
 * NOTE: If modifying this system state, please be sure to update the corresponding file in linear_slider_hardware_interface
 */ 


#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#ifndef __SERIAL_DEBUG__
#define __SERIAL_DEBUG__ 0
#endif

namespace slidersystem
{
	enum SystemStatus {
		SYSTEM_OK,
		SYSTEM_STANDBY,
		SYSTEM_CALIBRATING,
		E_STOP,
		NEG_LIM,
		POS_LIM,
	};
	
	struct DataInterface {
		slidersystem::SystemStatus system_status;
		double vel;
		int rpm;
	};

} // namespace system

#endif /* __SYSTEM_H__ */