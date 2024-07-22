/*
 * interrupts.h
 *
 * Created: 1/5/2024 5:14:42 PM
 *  Author: Luke Strohbehn
 */ 


#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include "system.h"


extern volatile bool neg_lim_switch_flag;
extern volatile bool pos_lim_switch_flag;
extern volatile bool e_stop_flag;

void emergency_stop_callback() {
	e_stop_flag = true;
}

void neg_lim_switch_callback() {
	neg_lim_switch_flag = true;
}

void pos_lim_switch_callback() {
	pos_lim_switch_flag = true;
}

#endif /* INTERRUPTS_H_ */