

#include <asm/jzsoc.h> 

#if 0
#define cim_power_off()		\
	do{				\	
		__camera_power_off();	\
	}while(0)	

#define cim_power_on() 		\
	do{				\
		__camera_power_on();	\
	}while(0)
#endif

void cim_power_off(void)
{
	__camera_power_off();
}

void cim_power_on(void)
{
	__camera_power_on();
}

