#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"

#define PIN_CS_N 	     (32*2+9) // (32*4+4)
#define PIN_RESET_N 	 (32*4+2) // (32*3+11)

//	#define SPEN		(32*4+0)       /*LCD_CS*/
//	#define SPCK		(32*3+11)       /*LCD_SCL*/
//	#define SPDA		(32*4+2)       /*LCD_SDA*/
//	#define LCD_RET 	(32*4+4)
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "driver/video/jz4760_lcd, please define SPI pins on your board."
#endif


static void Mcupanel_RegSet(unsigned short cmd, unsigned short data)
{
#if 1
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
#else
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0x00ff) >> 0);
#endif

	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xffff);
}

static void Mcupanel_Command(unsigned short cmd)
{
#if 1
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
#else
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8);

	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0x00ff) >> 0);
#endif
}

static void Mcupanel_Data(unsigned short data)
{
#if 1
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xffff);
#else
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xff00)>>8;

	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0x00ff);
#endif
}

static void LCDWriteCommand(unsigned short cmd)
{
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | ((cmd&0x00ff) >> 0);
}

static void LCDWriteData(unsigned short data)
{
#if 1
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xffff);
#else
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0xff00)>>8;

	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | (data&0x00ff);
#endif
}

static void ILI9325c()
{
	printk("init ILI9325c slcd \n");
	Mcupanel_RegSet(0x0001, 0x0000);
	Mcupanel_RegSet(0x0002, 0x0700);
	Mcupanel_RegSet(0x0003, 0x1090); // 1038	1088
	Mcupanel_RegSet(0x0004, 0x0000);
	Mcupanel_RegSet(0x0008, 0x0202); //0x0202
	Mcupanel_RegSet(0x0009, 0x0000);
	Mcupanel_RegSet(0x000A, 0x0000); // FMARK function
	Mcupanel_RegSet(0x000C, 0x0000); // RGB interface setting
	Mcupanel_RegSet(0x000D, 0x0000); // Frame marker Position
	Mcupanel_RegSet(0x000F, 0x0000); // RGB interface polarity
	//*************Power On sequence ****************//
	Mcupanel_RegSet(0x0010, 0x0000);
	Mcupanel_RegSet(0x0011, 0x0007);
	Mcupanel_RegSet(0x0012, 0x0000);
	Mcupanel_RegSet(0x0013, 0x0000);
	Mcupanel_RegSet(0x0007, 0x0001);
	mdelay(200);
	Mcupanel_RegSet(0x0010, 0x1290);
	Mcupanel_RegSet(0x0011, 0x0227);
	mdelay(50);
	Mcupanel_RegSet(0x0012, 0x001b);
	mdelay(50);
	Mcupanel_RegSet(0x0013, 0x0f00);
	Mcupanel_RegSet(0x0029, 0x0002); // VCOMH  32 25
	Mcupanel_RegSet(0x002B, 0x000e); // Set Frame Rate
	mdelay(50);
	Mcupanel_RegSet(0x0020, 0x0000); // GRAM horizontal Address
	Mcupanel_RegSet(0x0021, 0x0000); // GRAM Vertical Address
	// ----------- Adjust the Gamma   Curve ----------//
	Mcupanel_RegSet(0x0030, 0x0000);
	Mcupanel_RegSet(0x0031, 0x0406);
	Mcupanel_RegSet(0x0032, 0x0004);
	Mcupanel_RegSet(0x0035, 0x0305);
	Mcupanel_RegSet(0x0036, 0x0004);
	Mcupanel_RegSet(0x0037, 0x0207);
	Mcupanel_RegSet(0x0038, 0x0103);
	Mcupanel_RegSet(0x0039, 0x0707);
	Mcupanel_RegSet(0x003C, 0x0503);
	Mcupanel_RegSet(0x003D, 0x0004);
	//------------------ Set GRAM area ---------------//
	Mcupanel_RegSet(0x0050, 0x0000); // Horizontal GRAM Start Address
	Mcupanel_RegSet(0x0051, 0x00EF); // Horizontal GRAM End Address
	Mcupanel_RegSet(0x0052, 0x0000); // Vertical GRAM Start Address
	Mcupanel_RegSet(0x0053, 0x013F); // Vertical GRAM Start Address
	Mcupanel_RegSet(0x0060, 0xA700); // Gate Scan Line

	Mcupanel_RegSet(0x0061, 0x0001); // NDL,VLE, REV
	Mcupanel_RegSet(0x006A, 0x0000); // set scrolling line
	//-------------- Partial Display Control ---------//
	Mcupanel_RegSet(0x0080, 0x0000);
	Mcupanel_RegSet(0x0081, 0x0000);
	Mcupanel_RegSet(0x0082, 0x0000);
	Mcupanel_RegSet(0x0083, 0x0000);
	Mcupanel_RegSet(0x0084, 0x0000);
	Mcupanel_RegSet(0x0085, 0x0000);

	//-------------- Panel Control -------------------//
	Mcupanel_RegSet(0x0090, 0x0010);
	Mcupanel_RegSet(0x0092, 0x0600);

	Mcupanel_RegSet(0x0007, 0x0133); // 262K color and display ON

	Mcupanel_Command(0x22);
}

static void st7789s()
{
	printk("init st7789s slcd \n");
	Mcupanel_Command(0x11);
	mdelay (120);

	Mcupanel_Command(0x36);
	Mcupanel_Data(0x60);

	Mcupanel_Command(0x3a);
	Mcupanel_Data(0x05);
	//--------------------------------ST7789S Frame rate setting----------------------------------//
	Mcupanel_Command(0xb2);
	Mcupanel_Data(0x0c);
	Mcupanel_Data(0x0c);
	Mcupanel_Data(0x00);
	Mcupanel_Data(0x33);
	Mcupanel_Data(0x33);
	Mcupanel_Command(0xb7);
	Mcupanel_Data(0x35);
	//---------------------------------ST7789S Power setting--------------------------------------//
	Mcupanel_Command(0xbb);
	Mcupanel_Data(0x28);
	Mcupanel_Command(0xc0);
	Mcupanel_Data(0x2c);
	Mcupanel_Command(0xc2);
	Mcupanel_Data(0x01);
	Mcupanel_Command(0xc3);
	Mcupanel_Data(0x0b);
	Mcupanel_Command(0xc4);
	Mcupanel_Data(0x20);
	Mcupanel_Command(0xc6);
	Mcupanel_Data(0x0f);

	Mcupanel_Command(0xd0);
	Mcupanel_Data(0xa4);
	Mcupanel_Data(0xa1);
	//--------------------------------ST7789S gamma setting---------------------------------------//
	Mcupanel_Command(0xe0);
	Mcupanel_Data(0xd0);
	Mcupanel_Data(0x01);
	Mcupanel_Data(0x08);
	Mcupanel_Data(0x0f);
	Mcupanel_Data(0x11);
	Mcupanel_Data(0x2a);
	Mcupanel_Data(0x36);
	Mcupanel_Data(0x55);
	Mcupanel_Data(0x44);
	Mcupanel_Data(0x3a);
	Mcupanel_Data(0x0b);
	Mcupanel_Data(0x06);
	Mcupanel_Data(0x11);
	Mcupanel_Data(0x20);

	Mcupanel_Command(0xe1);
	Mcupanel_Data(0xd0);
	Mcupanel_Data(0x02);
	Mcupanel_Data(0x07);
	Mcupanel_Data(0x0a);
	Mcupanel_Data(0x0b);
	Mcupanel_Data(0x18);
	Mcupanel_Data(0x34);
	Mcupanel_Data(0x43);
	Mcupanel_Data(0x4a);
	Mcupanel_Data(0x2b);
	Mcupanel_Data(0x1b);
	Mcupanel_Data(0x1c);
	Mcupanel_Data(0x22);
	Mcupanel_Data(0x1f);

	//-----display window 240X320---------//
	Mcupanel_Command(0x2b); // h
	Mcupanel_Data(0x00);
	Mcupanel_Data(0x00);
	Mcupanel_Data(0x00);
	Mcupanel_Data(0xef);
	Mcupanel_Command(0x2a); // w
	Mcupanel_Data(0x00);
	Mcupanel_Data(0x00);
	Mcupanel_Data(0x01);
	Mcupanel_Data(0x3f);

	Mcupanel_Command(0x29);
	Mcupanel_Command(0x2c);
}


static void SlcdInit(void)
{
	st7789s();
	//ILI9325c();
}

#define __lcd_slcd_pin_init() \
do { \
	__gpio_as_func0(PIN_CS_N); \
	__gpio_as_output(PIN_CS_N); \
	__gpio_clear_pin(PIN_CS_N); \
\
	__gpio_as_output(PIN_RESET_N); \
	__gpio_set_pin(PIN_RESET_N); \
	mdelay(2); \
	__gpio_clear_pin(PIN_RESET_N); \
	mdelay(10); \
	__gpio_set_pin(PIN_RESET_N); \
	mdelay(10); \
\
	/* Configure SLCD module */ \
	REG_LCD_CTRL &= ~(LCD_CTRL_ENA|LCD_CTRL_DIS); /* disable lcdc */ \
	REG_SLCD_CTRL &= ~SLCD_CTRL_DMA_EN; /* disable slcd dma */ \
} while (0)

#define __lcd_slcd_special_on() \
do { \
	__lcd_slcd_pin_init(); \
	SlcdInit(); \
	/*__gpio_set_pin(PIN_CS_N);*/ \
	/*REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN;  /* slcdc dma enable */ \
	/* __slcd_set_data_8bit_x2(); */ \
} while (0)

#define __lcd_special_off() \
  do { \
	__gpio_as_output(GPIO_LCD_PWM); \
	__gpio_clear_pin(GPIO_LCD_PWM); \
  } while (0)



struct jz4760lcd_info jz4760_lcd_panel = {
	.panel = {
		.cfg = LCD_CFG_LCDPIN_SLCD |	// Underrun recover
			   LCD_CFG_NEWDES |			// 8words descriptor
			   LCD_CFG_MODE_SLCD,		// TFT Smart LCD panel
		// .slcd_cfg = SLCD_CFG_DWIDTH_8BIT_x2| SLCD_CFG_CWIDTH_8BIT|SLCD_CFG_CS_ACTIVE_LOW|SLCD_CFG_RS_CMD_LOW |SLCD_CFG_CLK_ACTIVE_FALLING| SLCD_CFG_TYPE_PARALLEL,
		.slcd_cfg = SLCD_CFG_DWIDTH_16BIT | SLCD_CFG_CWIDTH_16BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
		// .slcd_cfg = SLCD_CFG_DWIDTH_8BIT_x1 | SLCD_CFG_CWIDTH_8BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	// 16words burst, enable out FIFO underrun irq
	/* width,height,freq,hsync,vsync,elw,blw,efw,bfw */
		320, 240, 60, 0, 0, 0, 0, 0, 0,
	 },
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN |			// enable Foreground0
				   LCD_OSDC_F1EN,			// enable Foreground1
		.osd_ctrl = LCD_OSDCTRL_IPU | LCD_OSDCTRL_OSDBPP_15_16,	// enable ipu
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,				// set background color Black
		.colorkey0 = 0x80000000,			// disable colorkey
		.colorkey1 = 0x80000000,			// disable colorkey
		.alpha = 0xA0,						// alpha value
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
		.fg1 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
	 },
};
