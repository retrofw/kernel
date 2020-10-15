#ifndef __RTC_JZ_H__
#define __RTC_JZ_H__

//#define DEBUG 		1
#undef DEBUG

#ifdef DEBUG
#define dprintk(x...)	printk(x)
#define print_dbg(f, arg...) \
		printk("%s, %s[%d]:" f , __FUNCTION__, __FILE__, __LINE__ , ##arg )
#else
#define dprintk(x...)
#define print_dbg(n, arg...)
#endif


#ifdef DEBUG

static void print_rtc_time( struct rtc_time * tm )
{
	printk("%02d%02d-%02d:%02d:%02d-%d\n", tm->tm_mon, tm->tm_mday, 
	       tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_year);
	printk("sec:\t%d\n", tm->tm_sec);
	printk("min:\t%d\n", tm->tm_min);
	printk("hour:\t%d\n", tm->tm_hour);
	printk("mday:\t%d\n", tm->tm_mday);
	printk("mon:\t%d\n", tm->tm_mon);
	printk("year:\t%d\n", tm->tm_year);
	printk("wday:\t%d\n", tm->tm_wday);
	printk("yday:\t%d\n", tm->tm_yday);
	printk("isdst:\t%d\n", tm->tm_isdst);

}

static void print_rtc_registers( void )
{
	while ( !__rtc_write_ready() ) ; 
	printk("REG_RTC_RCR:\t 0x%8.8x\n", REG_RTC_RCR );
	printk("REG_RTC_RSR:\t 0x%8.8x\n", REG_RTC_RSR );
	printk("REG_RTC_RSAR:\t 0x%8.8x\n", REG_RTC_RSAR );
	printk("REG_RTC_RGR:\t 0x%8.8x\n", REG_RTC_RGR );
	printk("REG_RTC_HCR:\t 0x%8.8x\n", REG_RTC_HCR );
	printk("REG_RTC_HWFCR:\t 0x%8.8x\n", REG_RTC_HWFCR );
	printk("REG_RTC_HRCR:\t 0x%8.8x\n", REG_RTC_HRCR );
	printk("REG_RTC_HWCR:\t 0x%8.8x\n", REG_RTC_HWCR );
	printk("REG_RTC_HWRSR:\t 0x%8.8x\n", REG_RTC_HWRSR );
	printk("REG_RTC_HSPR:\t 0x%8.8x\n", REG_RTC_HSPR );
}

#define RTC_PRINT_REG	_IOR('p', 0x12, unsigned long)/* Set power down */
#endif /* #ifdef DEBUG */


/*
 * JZSOC ioctl calls that are permitted to the /dev/rtc interface
 */

#define RTC_ENABLED	_IO('p', 0x11)	/* enable rtc			*/
#define RTC_DISABLED	_IO('p', 0x12)	/* disable rtc			*/
#define RTC_ALM_ON	_IO('p', 0x13)	/* enable rtc			*/
#define RTC_ALM_OFF	_IO('p', 0x14)	/* disable rtc			*/
#define RTC_1HZIE_ON	_IO('p', 0x15)	/* 1Hz int. enable on		*/
#define RTC_1HZIE_OFF	_IO('p', 0x16)	/* ... off			*/

#define RTC_POWER_DOWN	_IOR('p', 0x11, unsigned long)/* Set power down */

/* Registers define */
/* RTC Control register */
#define RTC_AIE 	3	/* jz4740_06_rtc_spec.pdf, RTC Control Register */
#define RTC_1HZIE 	5	/* ... */
#define RTC_ALM_EN 	2	/* ... */
#define RTC_EN		0	/* ... */

#endif /* #define __RTC_JZ_H__ */
