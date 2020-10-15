#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
	#define PIN_CS_N 	(32*1+29)	/* a low voltage */
	#define PIN_RD_N 	(32*2+9)	/* LCD_DE: GP C9, a high voltage */
	#define PIN_RESET_N (32*5+10)	/* LCD_REV GP F10 */
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "Define special lcd pins for your platform."
#endif

static inline void CmdWrite(unsigned int cmd)
{
	while (REG_SLCD_STATE & SLCD_STATE_BUSY); /* wait slcd ready */
	udelay(30);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
}

static inline void DataWrite(unsigned int data)
{
	while (REG_SLCD_STATE & SLCD_STATE_BUSY); /* wait slcd ready */
//	udelay(30);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | data;
}


static inline void delay(long delay_time)
{
	long cnt;

//  delay_time *= (384/8);
	delay_time *= (43/8);

	for (cnt=0;cnt<delay_time;cnt++)
	{
		asm("nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
		    "nop\n"
			);
	}
}


/*---- LCD Initial ----*/
static void SlcdInit(void)
{
  delay(10000);
  CmdWrite(0x0301); //reset
  delay(10000);
  CmdWrite(0x0101);
  CmdWrite(0x0301);
  CmdWrite(0x0008);
  CmdWrite(0x2201); //reset
  CmdWrite(0x0000);
  CmdWrite(0x0080); //0x0020
  delay(10000);

  CmdWrite(0x2809);
  CmdWrite(0x1900);
  CmdWrite(0x2110);
  CmdWrite(0x1805);
  CmdWrite(0x1E01);
  CmdWrite(0x1847);
  delay(1000);
  CmdWrite(0x1867);
  delay(10000);
  CmdWrite(0x18F7);
  delay(10000);
  CmdWrite(0x2100);
  CmdWrite(0x2809);
  CmdWrite(0x1a05);
  CmdWrite(0x1900);
  CmdWrite(0x1f64);
  CmdWrite(0x2070);
  CmdWrite(0x1e81);
  CmdWrite(0x1b01);

  CmdWrite(0x0200);
  CmdWrite(0x0504); // y address increcement
  CmdWrite(0x0D00); // *240
  CmdWrite(0x1D08);
  CmdWrite(0x2300);
  CmdWrite(0x2D01);
  CmdWrite(0x337F);
  CmdWrite(0x3400);
  CmdWrite(0x3501);
  CmdWrite(0x3700);
  CmdWrite(0x42ef); // x start from 239
  CmdWrite(0x4300);
  CmdWrite(0x4400); // y start from 0
  CmdWrite(0x4500);
  CmdWrite(0x46EF);
  CmdWrite(0x4700);
  CmdWrite(0x4800);
  CmdWrite(0x4901);
  CmdWrite(0x4A3F);
  CmdWrite(0x4B00);
  CmdWrite(0x4C00);
  CmdWrite(0x4D00);
  CmdWrite(0x4E00);
  CmdWrite(0x4F00);
  CmdWrite(0x5000);
  CmdWrite(0x7600);
  CmdWrite(0x8600);
  CmdWrite(0x8720);
  CmdWrite(0x8802);
  CmdWrite(0x8903);
  CmdWrite(0x8D40);
  CmdWrite(0x8F05);
  CmdWrite(0x9005);
  CmdWrite(0x9144);
  CmdWrite(0x9244);
  CmdWrite(0x9344);
  CmdWrite(0x9433);
  CmdWrite(0x9505);
  CmdWrite(0x9605);
  CmdWrite(0x9744);
  CmdWrite(0x9844);
  CmdWrite(0x9944);
  CmdWrite(0x9A33);
  CmdWrite(0x9B33);
  CmdWrite(0x9C33);
  //==> SETP 3
  CmdWrite(0x0000);
  CmdWrite(0x01A0);
  CmdWrite(0x3B01);

  CmdWrite(0x2809);
  delay(1000);
  CmdWrite(0x1900);
  delay(1000);
  CmdWrite(0x2110);
  delay(1000);
  CmdWrite(0x1805);
  delay(1000);
  CmdWrite(0x1E01);
  delay(1000);
  CmdWrite(0x1847);
  delay(1000);
  CmdWrite(0x1867);
  delay(1000);
  CmdWrite(0x18F7);
  delay(1000);
  CmdWrite(0x2100);
  delay(1000);
  CmdWrite(0x2809);
  delay(1000);
  CmdWrite(0x1A05);
  delay(1000);
  CmdWrite(0x19E8);
  delay(1000);
  CmdWrite(0x1F64);
  delay(1000);
  CmdWrite(0x2045);
  delay(1000);
  CmdWrite(0x1E81);
  delay(1000);
  CmdWrite(0x1B09);
  delay(1000);
  CmdWrite(0x0020);
  delay(1000);
  CmdWrite(0x0120);
  delay(1000);

  CmdWrite(0x3B01);
  delay(1000);

  /* Set Window(239,319), Set Cursor(239,319) */
  CmdWrite(0x0510);
  CmdWrite(0x01C0);
  CmdWrite(0x4500);
  CmdWrite(0x46EF);
  CmdWrite(0x4800);
  CmdWrite(0x4700);
  CmdWrite(0x4A3F);
  CmdWrite(0x4901);
  CmdWrite(0x42EF);
  CmdWrite(0x443F);
  CmdWrite(0x4301);
}

#define __lcd_slcd_pin_init() \
	do { \
		__gpio_as_output(PIN_RD_N); /* RD#: LCD_REV */ \
		__gpio_as_output(PIN_RESET_N); /* RESET#: LCD_SPL */ \
		__gpio_set_pin(PIN_RD_N); /*set read signal high */ \
		__gpio_set_pin(PIN_RESET_N); \
		mdelay(100); \
		__gpio_clear_pin(PIN_RESET_N); \
		mdelay(100); \
		__gpio_set_pin(PIN_RESET_N); \
		/* Configure SLCD module */ \
		REG_LCD_CTRL &= ~(LCD_CTRL_ENA|LCD_CTRL_DIS); /* disable lcdc */ \
		REG_LCD_CFG = LCD_CFG_LCDPIN_SLCD | 0x0D; /* LCM */ \
		REG_SLCD_CTRL &= ~SLCD_CTRL_DMA_EN; /* disable slcd dma */ \
		REG_SLCD_CFG = SLCD_CFG_DWIDTH_16BIT | SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL; \
		REG_LCD_REV = 0x04;	/* lcd clock??? */ \
		printk("Fuwa test, pixclk divide REG_LCD_REV=0x%08x\n", REG_LCD_REV); \
}while (0)

#define __lcd_slcd_special_on() \
	do { \
		__lcd_slcd_pin_init(); \
		SlcdInit(); \
		REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; /* slcdc dma enable */ \
	} while (0)

struct jz4760lcd_info jz4760_lcd_panel = {
	.panel = {
		.cfg = LCD_CFG_LCDPIN_SLCD |	// Underrun recover
			   LCD_CFG_NEWDES |			// 8words descriptor
			   LCD_CFG_MODE_SLCD,	// TFT Smart LCD panel
		.slcd_cfg = SLCD_CFG_DWIDTH_16BIT | SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	// 16words burst, enable out FIFO underrun irq
		240, 320, 60, 0, 0, 0, 0, 0, 0,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,			// enable Foreground0
				   // LCD_OSDC_F1EN,		// enable Foreground1
		.osd_ctrl = 0,						// disable ipu
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,				// set background color Black
		.colorkey0 = 0,						// disable colorkey
		.colorkey1 = 0,						// disable colorkey
		.alpha = 0xA0,						// alpha value
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {32, 0, 0, 240, 320},		// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 240, 320},		// bpp, x, y, w, h
	},
};
