/*
 * system.h
 *
 * Created: 1/5/2024 10:07:56 PM
 *  Author: Luke Strohbehn
 */ 


#ifndef __SYSTEM_H__
#define __SYSTEM_H__

namespace slidersystem
{
	enum SystemStatus {
		SYSTEM_OK,
		SYSTEM_STANDBY,
		NEG_LIM,
		POS_LIM,
		E_STOP,
	};

} // namespace system

#endif /* __SYSTEM_H__ */