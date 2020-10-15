#ifndef __ASM_JZ4740_LEO_H__
#define __ASM_JZ4740_LEO_H__

/*
 * Define your board specific codes here !!!
 */

/*====================================================================== 
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq: 12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */


/*====================================================================== 
 * GPIO
 */
#define GPIO_DISP_OFF_N		100
#define GPIO_SD_VCC_EN_N	119
#define GPIO_SD_CD_N		120
#define GPIO_SD_WP		111

/*====================================================================== 
 * MMC/SD
 */

#define MSC_WP_PIN		GPIO_SD_WP
#define MSC_HOTPLUG_PIN		GPIO_SD_CD_N
#define MSC_HOTPLUG_IRQ		(IRQ_GPIO_0 + GPIO_SD_CD_N)

#define __msc_init_io()				\
do {						\
	__gpio_as_output(GPIO_SD_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD_CD_N);		\
} while (0)

#define __msc_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD_VCC_EN_N);	\
} while (0)

#define __msc_card_detected(s)			\
({						\
	int detected = 1;			\
	__gpio_as_input(GPIO_SD_CD_N);		\
	if (__gpio_get_pin(GPIO_SD_CD_N))	\
		detected = 0;			\
	detected;				\
})

#endif /* __ASM_JZ4740_BOARD_LEO_H__ */
