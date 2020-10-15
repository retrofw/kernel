/* Set registers of smart lcd acording to the following routines
 * Note: BUS width and CMD width and register value width
 * This example: BUS is 8, 9, 16 or 18-bit; CMD and DATA is 16-bit
 * Configure SLCD module to initialize smart lcd registers

	switch (bus) {
	case 8:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_8_x2 
			| SLCD_CFG_CWIDTH_8BIT | SLCD_CFG_CS_ACTIVE_LOW 
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING 
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_8bit();
		break;
	case 9:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_8_x2
			| SLCD_CFG_CWIDTH_8BIT | SLCD_CFG_CS_ACTIVE_LOW 
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING 
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_9bit();
		break;
	case 16:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_16
			| SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_16bit();
		break;
	case 18:
		REG_SLCD_CFG = SLCD_CFG_BURST_8_WORD | SLCD_CFG_DWIDTH_18
			| SLCD_CFG_CWIDTH_18BIT | SLCD_CFG_CS_ACTIVE_LOW 
			| SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING 
			| SLCD_CFG_TYPE_PARALLEL;
		__gpio_as_slcd_18bit();
		break;
	}
 
   static void Mcupanel_RegSet(unsigned int cmd, unsigned int data)
   {
 	switch (bus) {
 	case 8:
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xffff);
 		break;
 	case 9:
 		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
 		data = ((data << 6) & 0xfc0000) | ((data << 4) & 0xfc00) | ((data << 2) & 0xfc);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_DATA | data;
 		break;
 	case 16:
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | (cmd&0xffff);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xffff);
 		break;
 	case 18:
 		cmd = ((cmd & 0xff) << 1) | ((cmd & 0xff00) << 2); 	
 		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_DATA | ((data<<6)&0xfc0000)|((data<<4)&0xfc00) | ((data<<2)&0xfc);
 		break;
 	default:
 		printk("Don't support %d bit Bus\n", jzfb.bus );
 		break;
 	}
  }
 
  static void Mcupanel_Command(unsigned int cmd) {
 	switch (bus) {
 	case 8:
 	case 9:
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0);
 		break;
 	case 16:
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | (cmd&0xffff);
 		break;
 	case 18:
 		while (REG_SLCD_STATE & SLCD_STATE_BUSY);
 		REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) << 2) | ((cmd&0xff) << 1);
 		break;
 	default:
 		printk("Don't support %d bit Bus\n", jzfb.bus );
 		break;
 	}
  }

  *Display----------------------------------------
  Note: BUS and BPP, send data to gram data register to display
        BUS: 8, 9, 16 or 18-bit; BPP: 8, 16, 18-bit
	switch (bus) {
	case 8:
		switch (bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15:
		case 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x2;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x3;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	case 9:
		switch (bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15 ... 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x2;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_9_x2;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	case 16:
		switch (bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15 ... 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_16;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x3;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	case 18:
		switch (bpp) {
		case 8:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15:
		case 16:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_16;
			break;
		case 17 ... 32:
			REG_SLCD_CFG &= ~SLCD_CFG_DWIDTH_MASK;
			REG_SLCD_CFG |= SLCD_CFG_DWIDTH_18;
			break;
		default:
			printk("The BPP %d is not supported\n", jzfb.bpp);
			break;
		}
		break;
	default:
		printk("Error: The BUS %d is not supported\n", jzfb.bus);
		break;
	}
	dprintk("SLCD_CFG=0x%x\n", REG_SLCD_CFG);
}
 ************************************************************************************************/

#ifndef __JZ_KGM_SPF5420A_H__
#define __JZ_KGM_SPF5420A_H__

#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4750_SLCD_KGM701A3_TFT_SPFD5420A) || defined(CONFIG_JZ4760_SLCD_KGM701A3_TFT_SPFD5420A)
#define WR_GRAM_CMD	0x0202

#if defined(CONFIG_JZ4750_FUWA)
#define PIN_CS_N 	(32*3+24)	// Chip select 	//GPD24;
#define PIN_RESET_N 	(32*5+6)	/* LCD_REV GPF6 */
#elif defined(CONFIG_JZ4750D_FUWA1)
#define PIN_CS_N 	(32*3+24)	// Chip select 	//GPD24;
#define PIN_RESET_N 	(32*4+3)	/* LCD_REV GPF6 */
#elif defined(CONFIG_JZ4770_PISCES)
#define PIN_CS_N 	(32*3+24)	// Chip select 	//GPD24;
#define PIN_RESET_N 	(32*4+3)	/* LCD_REV GPF6 */
#else
#error "Define special lcd pins for your platform."
#endif

/* Sent a command with data (18-bit bus, 16-bit index, 16-bit register value) */
static void Mcupanel_RegSet(unsigned int cmd, unsigned int data)
{
	cmd = ((cmd & 0xff) << 1) | ((cmd & 0xff00) << 2);
	data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
	data = ((data<<6)&0xfc0000)|((data<<4)&0xfc00) | ((data<<2)&0xfc);
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | data;
}

/* Sent a command without data  (18-bit bus, 16-bit index) */
static void Mcupanel_Command(unsigned int cmd) {
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) << 2) | ((cmd&0xff) << 1);
}

/* Set the start address of screen, for example (0, 0) */
void Mcupanel_SetAddr(u32 x, u32 y) //u32
{
	Mcupanel_RegSet(0x200,x) ;
	udelay(1);
	Mcupanel_RegSet(0x201,y) ;
	udelay(1);
	Mcupanel_Command(0x202);

}

#undef __lcd_special_pin_init
#define __lcd_special_pin_init() \
do {	\
	__gpio_as_output(PIN_CS_N);	\
	__gpio_as_output(PIN_RESET_N);	\
	__gpio_clear_pin(PIN_CS_N); /* Clear CS */	\
	mdelay(100);	\
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(10);	\
	__gpio_clear_pin(PIN_RESET_N);	\
	mdelay(10);	\
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(100);	\
} while(0)


#define GAMMA()	\
do {	\
	Mcupanel_RegSet(0x0300,0x0101);	\
	Mcupanel_RegSet(0x0301,0x0b27);	\
	Mcupanel_RegSet(0x0302,0x132a);	\
	Mcupanel_RegSet(0x0303,0x2a13);	\
	Mcupanel_RegSet(0x0304,0x270b);	\
	Mcupanel_RegSet(0x0305,0x0101);	\
	Mcupanel_RegSet(0x0306,0x1205);	\
	Mcupanel_RegSet(0x0307,0x0512);	\
	Mcupanel_RegSet(0x0308,0x0005);	\
	Mcupanel_RegSet(0x0309,0x0003);	\
	Mcupanel_RegSet(0x030a,0x0f04);	\
	Mcupanel_RegSet(0x030b,0x0f00);	\
	Mcupanel_RegSet(0x030c,0x000f);	\
	Mcupanel_RegSet(0x030d,0x040f);	\
	Mcupanel_RegSet(0x030e,0x0300);	\
	Mcupanel_RegSet(0x030f,0x0500);	\
	/*** secorrect gamma2 ***/	\
	Mcupanel_RegSet(0x0400,0x3500);	\
	Mcupanel_RegSet(0x0401,0x0001);	\
	Mcupanel_RegSet(0x0404,0x0000);	\
	Mcupanel_RegSet(0x0500,0x0000);	\
	Mcupanel_RegSet(0x0501,0x0000);	\
	Mcupanel_RegSet(0x0502,0x0000);	\
	Mcupanel_RegSet(0x0503,0x0000);	\
	Mcupanel_RegSet(0x0504,0x0000);	\
	Mcupanel_RegSet(0x0505,0x0000);	\
	Mcupanel_RegSet(0x0600,0x0000);	\
	Mcupanel_RegSet(0x0606,0x0000);	\
	Mcupanel_RegSet(0x06f0,0x0000);	\
	Mcupanel_RegSet(0x07f0,0x5420);	\
	Mcupanel_RegSet(0x07f3,0x288a);	\
	Mcupanel_RegSet(0x07f4,0x0022);	\
	Mcupanel_RegSet(0x07f5,0x0001);	\
	Mcupanel_RegSet(0x07f0,0x0000);	\
} while(0)

#define SlcdInit()	\
do {      \
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(10);	\
	__gpio_clear_pin(PIN_RESET_N);	\
	mdelay(10);	\
	__gpio_set_pin(PIN_RESET_N);	\
	mdelay(100);	\
	Mcupanel_RegSet(0x0600, 0x0001);   /*soft reset*/	\
	mdelay(10); 		\
	Mcupanel_RegSet(0x0600, 0x0000);   /*soft reset*/	\
	mdelay(10);						\
	Mcupanel_RegSet(0x0606,0x0000);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0007,0x0001);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0110,0x0001);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0100,0x17b0);	\
	Mcupanel_RegSet(0x0101,0x0147);	\
	Mcupanel_RegSet(0x0102,0x019d);	\
	Mcupanel_RegSet(0x0103,0x8600);	\
	Mcupanel_RegSet(0x0281,0x0010);	\
	udelay(10);	\
	Mcupanel_RegSet(0x0102,0x01bd);	\
	udelay(10);	\
	/************initial************/\
	Mcupanel_RegSet(0x0000,0x0000);	\
	Mcupanel_RegSet(0x0001,0x0000);	\
	Mcupanel_RegSet(0x0002,0x0400);	\
	Mcupanel_RegSet(0x0003,0x12b8); /*up:0x1288 down:0x12B8 left:0x1290 right:0x12A0*/ \
	Mcupanel_RegSet(0x0006,0x0000);	\
	Mcupanel_RegSet(0x0008,0x0503);	\
	Mcupanel_RegSet(0x0009,0x0001);	\
	Mcupanel_RegSet(0x000b,0x0010);	\
	Mcupanel_RegSet(0x000c,0x0000);	\
	Mcupanel_RegSet(0x000f,0x0000);	\
	Mcupanel_RegSet(0x0007,0x0001);	\
	Mcupanel_RegSet(0x0010,0x0010);	\
	Mcupanel_RegSet(0x0011,0x0202);	\
	Mcupanel_RegSet(0x0012,0x0300);	\
	Mcupanel_RegSet(0x0020,0x021e);	\
	Mcupanel_RegSet(0x0021,0x0202);	\
	Mcupanel_RegSet(0x0022,0x0100);	\
	Mcupanel_RegSet(0x0090,0x0000);	\
	Mcupanel_RegSet(0x0092,0x0000);	\
	Mcupanel_RegSet(0x0100,0x16b0);	\
	Mcupanel_RegSet(0x0101,0x0147);	\
	Mcupanel_RegSet(0x0102,0x01bd);	\
	Mcupanel_RegSet(0x0103,0x2c00);	\
    	Mcupanel_RegSet(0x0107,0x0000);	\
	Mcupanel_RegSet(0x0110,0x0001);	\
	Mcupanel_RegSet(0x0210,0x0000);	\
	Mcupanel_RegSet(0x0211,0x00ef);	\
	Mcupanel_RegSet(0x0212,0x0000);	\
	Mcupanel_RegSet(0x0213,0x018f);	\
	Mcupanel_RegSet(0x0280,0x0000);	\
	Mcupanel_RegSet(0x0281,0x0001);	\
	Mcupanel_RegSet(0x0282,0x0000);	\
	GAMMA();	\
 	Mcupanel_RegSet(0x0007,0x0173);	\
	Mcupanel_Command(0x0202);                  /*Write Data to GRAM	*/ \
	udelay(10);\
	Mcupanel_SetAddr(0,0);\
	udelay(100);\
} while(0)

/*---- LCD Initial ----*/
#undef __lcd_slcd_pin_init
#define __lcd_slcd_pin_init()						\
	do {								\
		__lcd_special_pin_init();				\
}while (0)

#undef __lcd_slcd_special_on

#if defined(CONFIG_SOC_JZ4770)
#define __lcd_slcd_special_on()						\
	do {								\
		__lcd_slcd_pin_init();					\
		SlcdInit();						\
		REG_SLCD_CTRL &= ~(1 << 2); /* slcdc dma enable */		\
	} while (0)

#else
#define __lcd_slcd_special_on()						\
	do {								\
		__lcd_slcd_pin_init();					\
		SlcdInit();						\
		REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; /* slcdc dma enable */ \
	} while (0)
#endif

#define __init_slcd_bus()\
do{\
	__slcd_set_data_18bit();\
	__slcd_set_cmd_18bit();\
	__slcd_set_cs_low();\
	__slcd_set_rs_low();\
	__slcd_set_clk_falling();\
	__slcd_set_parallel_type();\
}while(0)
#endif	/* #if CONFIG_JZ4750_SLCD_KGM701A3_TFT_SPFD5420A */

#endif  /* __JZ_KGM_SPF5420A_H__ */
