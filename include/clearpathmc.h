/*
 * ClearPathMC.h
 *
 * Created: 9/21/2023 6:04:25 PM
 *  Author: Luke Strohbehn
 */ 


#ifndef CLEARPATHMC_H_
#define CLEARPATHMC_H_

class ClearPathMC {
	public:
		void set_position(float pos);
		void set_velocity(float vel);
		
		void stop();
};



#endif /* CLEARPATHMC_H_ */