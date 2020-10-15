

#include <asm/jzsoc.h>

//#define GPIO_CAMERA_RST	(32*4+8) /*GPE8 mclk*/

void cim_power_off(void)
{
	__cpm_stop_cim();
}

void cim_power_on(void)
{
	__cpm_start_cim();
}

