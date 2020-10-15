#include <asm/jzsoc.h> 

void cim_power_off(void)
{
	__camera_power_off();
	cpm_stop_clock(CGM_CIM);
}

void cim_power_on(void)
{
	__camera_power_on();
	cpm_start_clock(CGM_CIM);
}




