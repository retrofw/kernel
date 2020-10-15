#ifndef __LINUX_GWTC9XXXB_TS_H__
#define __LINUX_GWTC9XXXB_TS_H__

#define SCREEN_MAX_X    800
#define SCREEN_MAX_Y    480
#define PRESS_MAX       255

#define GWTC9XXXB_NAME	"gwtc9xxxb_ts"

struct gwtc9xxxb_ts_platform_data{
	u16	intr;		/* irq number	*/
};

enum gwtc9xxxb_ts_regs {
	GWTC9XXXB_REG_SLEEP	= 0x0a,	/* Sleep reg	*/	
};

//FGWTC9XXXB_REG_SLEEP	
#define     SLEEP_MODE 0x08
#endif
