#include <asm/jzsoc.h>

void cim_power_off(void)
{
  	cpm_stop_clock(CGM_CIM);
}

void cim_power_on(void)
{
       cpm_stop_clock(CGM_CIM);
       cpm_set_clock(CGU_CIMCLK,24000000);
       cpm_start_clock(CGM_CIM);
}




