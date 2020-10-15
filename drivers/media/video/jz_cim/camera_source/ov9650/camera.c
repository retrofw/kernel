

#include <asm/jzsoc.h>

extern void camera_clk_init(void);


/* gpio init */
#if defined(CONFIG_JZ4750_APUS) || defined(CONFIG_JZ4750D_FUWA1) || defined(CONFIG_JZ4750_AQUILA)/* board APUS */
#define GPIO_CAMERA_RST         (32*4+8) /* CIM_MCLK as reset */
#elif defined(CONFIG_JZ4760_F4760) /* JZ4760 FPGA */
#define GPIO_CAMERA_RST         (32*1+9) /* CIM_MCLK as reset */
#elif defined(CONFIG_JZ4760_LEPUS)
#define GPIO_CAMERA_RST         (32*1 + 26) /* GPB26 */
#else
#error "ov9650/camera.c , please define camera for your board."
#endif


void camera_powerdown() {;}
void camera_powerup()	{;}



void camera_power_init()
{
#if defined(CONFIG_JZ4750_AQUILA)
#error  "ov9650/camera.c ,plesase wite a new camera_power_init()"
#endif
}

void camera_reset(void)
{
#if  defined(CONFIG_JZ4750_AQUILA)
#error  "ov9650/camera.c ,plesase wite a new camera_reset()"
#else
  __gpio_as_output(GPIO_CAMERA_RST);
  __gpio_set_pin(GPIO_CAMERA_RST);
  mdelay(50);
  __gpio_clear_pin(GPIO_CAMERA_RST);
#endif

}

void camera_hw_init(void)
{
	camera_reset();//mclk_pin used as reset_pin, so make the mclk_pin as gpio first

	//1.cim poewerdowen pin control
	camera_powerup();

	//2.cim power init
	camera_power_init();

	//3.cim clock init
	//if mclk pin is not used for clk ,init it befor camera_clk_init()!
	camera_clk_init();

	//4.cim reset pin  control
	camera_reset();

}

int camera_set_init(void){return 1;}
int camera_set_parm(unsigned int id,int32_t parm) {return 1;}
int camera_set_parm2(unsigned int id,int32_t parm1,int32_t parm2) {return 1;}
int camera_set_preview(int width, int height, const char *format) {return 1;}
int camera_set_capture(int width, int height, const char *format) {return 1;}




