/*
 * PCF8563 Real Time Clock interface for Linux
 *
 * It only support 24Hour Mode, And the stored values are in BCD format.
 * The alarm register is start at minute reg, no second alarm register.
 */

//#include <linux/config.h>
#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/spinlock.h>

#include <linux/rtc.h>			/* get the user-level API */
#include <asm/system.h>
#include <asm/jzsoc.h>

/**********************************************************************
 * register summary
 **********************************************************************/
#define RTC_SECONDS		2
#define RTC_MINUTES		3
#define RTC_HOURS		4
#define RTC_DAY_OF_MONTH	5
#define RTC_DAY_OF_WEEK		6
#define RTC_MONTH		7
#define RTC_YEAR		8

#define RTC_MINUTES_ALARM	9
#define RTC_HOURS_ALARM		0x0a
#define RTC_DAY_ALARM           0x0b
#define RTC_WEEKDAY_ALARM	0x0c

/* control registers - Moto names
 */
#define RTC_CONTROL             0x00
#define RTC_STATUS              0x01
#define RTC_CLKOUT		0x0d
#define RTC_TIMERCTL		0x0e
#define RTC_TIMERCOUNTDOWN	0x0f


/* example: !(CMOS_READ(RTC_CONTROL) & RTC_DM_BINARY) 
 * determines if the following two #defines are needed
 */
#ifndef BCD2BIN
#define BCD2BIN(val) (((val) & 0x0f) + ((val) >> 4) * 10)
#endif

#ifndef BIN2BCD
#define BIN2BCD(val) ((((val) / 10) << 4) + (val) % 10)
#endif

extern spinlock_t rtc_lock;
extern void i2c_open(void);
extern void i2c_close(void);
extern int i2c_read(unsigned char device, unsigned char *buf,
		     unsigned char address, int count);
extern int i2c_write(unsigned char device, unsigned char *buf,
		      unsigned char address, int count);
/*
 *	We sponge a minor off of the misc major. No need slurping
 *	up another valuable major dev number for this. If you add
 *	an ioctl, make sure you don't conflict with SPARC's RTC
 *	ioctls.
 */

static int rtc_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg);


static void get_rtc_time (struct rtc_time *rtc_tm);
static void get_rtc_alm_time (struct rtc_time *alm_tm);

/*
 * rtc_status is never changed by rtc_interrupt, and ioctl/open/close is
 * protected by the big kernel lock. However, ioctl can still disable the timer
 * in rtc_status and then with del_timer after the interrupt has read
 * rtc_status but before mod_timer is called, which would then reenable the
 * timer (but you would need to have an awful timing before you'd trip on it)
 */
static unsigned long rtc_status = 0;	/* bitmapped status byte.	*/

/*
 *	If this driver ever becomes modularised, it will be really nice
 *	to make the epoch retain its value across module reload...
 */
static unsigned int epoch = 1900;
static const unsigned char days_in_mo[] = 
{0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static unsigned char rtcframe[16];

static void read_rtcframe(void)
{
	i2c_open();
	i2c_read(0x51, rtcframe, 0, 16);
	i2c_close();
}

static void write_rtcframe(void)
{
	i2c_open();
	i2c_write(0x51, rtcframe, 0, 16);
	i2c_close();
}

static void write_rtc(unsigned char addr, unsigned char val)
{
	volatile unsigned char v = val;
	i2c_open();
	i2c_write(0x51, (unsigned char *)&v, addr, 1);
	i2c_close();
}

static unsigned char read_rtc(unsigned char addr)
{
	volatile unsigned char v;
	i2c_open();
	i2c_read(0x51, (unsigned char *)&v, addr, 1);
	i2c_close();
	return v;
}

static void CMOS_WRITE(unsigned char addr, unsigned char val) 
{
	rtcframe[addr] = val; 
}

static unsigned char CMOS_READ(unsigned char addr) 
{
	return rtcframe[addr]; 
}

static void get_rtc_time(struct rtc_time *rtc_tm)
{
	unsigned char sec,mon,mday,wday,year,hour,min;

	/*
	 * Only the values that we read from the RTC are set. We leave
	 * tm_wday, tm_yday and tm_isdst untouched. Even though the
	 * RTC has RTC_DAY_OF_WEEK, we ignore it, as it is only updated
	 * by the RTC when initially set to a non-zero value.
	 */

	spin_lock_irq(&rtc_lock);
	read_rtcframe();
	sec	= CMOS_READ(RTC_SECONDS) & ~0x80;
	min	= CMOS_READ(RTC_MINUTES) & ~0x80;
	hour	= CMOS_READ(RTC_HOURS) & ~0xc0;
	mday	= CMOS_READ(RTC_DAY_OF_MONTH) & ~0xc0;
	wday	= CMOS_READ(RTC_DAY_OF_WEEK) & ~0xf8;
	mon	= CMOS_READ(RTC_MONTH) & ~0xe0;
	year	= CMOS_READ(RTC_YEAR)  ;

	rtc_tm->tm_sec = BCD2BIN(sec);
	rtc_tm->tm_min = BCD2BIN(min);
	rtc_tm->tm_hour = BCD2BIN(hour);
	rtc_tm->tm_mday = BCD2BIN(mday);
	rtc_tm->tm_wday = wday;
	/* Don't use centry, but start from year 1970 */
	rtc_tm->tm_mon = BCD2BIN(mon);
	year = BCD2BIN(year);
	if ((year += (epoch - 1900)) <= 69)
		year += 100;
	rtc_tm->tm_year = year;

	spin_unlock_irq(&rtc_lock);


	/*
	 * Account for differences between how the RTC uses the values
	 * and how they are defined in a struct rtc_time;
	 */
	rtc_tm->tm_mon--;
}

static void get_rtc_alm_time(struct rtc_time *alm_tm)
{ 
	unsigned char sec, min, hour;

	/*
	 * Only the values that we read from the RTC are set. That
	 * means only tm_hour, tm_min, and tm_sec.
	 */
	spin_lock_irq(&rtc_lock);
	read_rtcframe();
	sec = 0;
	min	= CMOS_READ(RTC_MINUTES_ALARM);
	hour	= CMOS_READ(RTC_HOURS_ALARM);

	alm_tm->tm_sec = sec;//not set sec
	alm_tm->tm_min = BCD2BIN(min);
	alm_tm->tm_hour = BCD2BIN(hour);

	spin_unlock_irq(&rtc_lock);
}

static int rtc_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     unsigned long arg)
{
	struct rtc_time wtime; 
	switch (cmd) {
	case RTC_ALM_READ:	/* Read the present alarm time */
	{
		/*
		 * This returns a struct rtc_time. Reading >= 0xc0
		 * means "don't care" or "match all". Only the tm_hour,
		 * tm_min, and tm_sec values are filled in.
		 */

		get_rtc_alm_time(&wtime);
		break; 
	}
	case RTC_ALM_SET:	/* Store a time into the alarm */
	{
		unsigned char hrs, min, sec;
		struct rtc_time alm_tm;

		if (copy_from_user(&alm_tm, (struct rtc_time*)arg,
				   sizeof(struct rtc_time)))
			return -EFAULT;

		hrs = alm_tm.tm_hour;
		min = alm_tm.tm_min;
		sec = alm_tm.tm_sec;

	

		if (hrs >= 24)
			return -EINVAL;

		hrs = BIN2BCD(hrs);

		if (min >= 60)
			return -EINVAL;

		min = BIN2BCD(min);

		if (sec >= 60)
			return -EINVAL;

		spin_lock_irq(&rtc_lock);
		read_rtcframe();
		CMOS_WRITE(RTC_HOURS_ALARM, hrs | 0x80);
		CMOS_WRITE(RTC_MINUTES_ALARM, min | 0x80);

		CMOS_WRITE(RTC_DAY_ALARM, CMOS_READ(RTC_DAY_ALARM) | 0x80); 
		CMOS_WRITE(RTC_WEEKDAY_ALARM, CMOS_READ(RTC_WEEKDAY_ALARM) | 0x80);
		CMOS_WRITE(RTC_STATUS, CMOS_READ(RTC_STATUS) | 0x02);/*open alarm int*/
		write_rtcframe();
		spin_unlock_irq(&rtc_lock);
		break;
	}
	case RTC_RD_TIME:	/* Read the time/date from RTC	*/
	{
		get_rtc_time(&wtime);
		break;
	}
	case RTC_SET_TIME:	/* Set the RTC */
	{
		struct rtc_time rtc_tm;
		unsigned char mon, day, hrs, min, sec, leap_yr, date;
		unsigned int yrs;
//		unsigned char ctr;

		if (!capable(CAP_SYS_TIME))
			return -EACCES;

		if (copy_from_user(&rtc_tm, (struct rtc_time*)arg,
				   sizeof(struct rtc_time)))
			return -EFAULT;


		yrs = rtc_tm.tm_year + 1900;
		mon = rtc_tm.tm_mon + 1;   /* tm_mon starts at zero */
		day = rtc_tm.tm_wday;
		date = rtc_tm.tm_mday;
		hrs = rtc_tm.tm_hour;
		min = rtc_tm.tm_min;
		sec = rtc_tm.tm_sec;


		if (yrs < 1970)
			return -EINVAL;

		leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));

		if ((mon > 12) || (date == 0))
			return -EINVAL;

		if (date > (days_in_mo[mon] + ((mon == 2) && leap_yr)))
			return -EINVAL;
			
		if ((hrs >= 24) || (min >= 60) || (sec >= 60))
			return -EINVAL;

		if ((yrs -= epoch) > 255)    /* They are unsigned */
			return -EINVAL;

		spin_lock_irq(&rtc_lock);
		/* These limits and adjustments are independant of
		 * whether the chip is in binary mode or not.
		 */
		if (yrs > 169) {
			spin_unlock_irq(&rtc_lock);
			return -EINVAL;
		}

		if (yrs >= 100)
			yrs -= 100;

		min = BIN2BCD(min);
		sec = BIN2BCD(sec);
		hrs = BIN2BCD(hrs);
		mon = BIN2BCD(mon);
		yrs = BIN2BCD(yrs);
		date = BIN2BCD(date);
 	
		read_rtcframe();
		CMOS_WRITE(RTC_SECONDS, sec );
		CMOS_WRITE(RTC_MINUTES, min);
		CMOS_WRITE(RTC_HOURS, hrs);
		CMOS_WRITE(RTC_DAY_OF_MONTH, date);
		CMOS_WRITE(RTC_DAY_OF_WEEK, day);
		CMOS_WRITE(RTC_MONTH, mon);
		CMOS_WRITE(RTC_YEAR, yrs);
		write_rtcframe();

		spin_unlock_irq(&rtc_lock);
		return 0;
	}
	case RTC_EPOCH_READ:	/* Read the epoch.	*/
	{
		return put_user (epoch, (unsigned long *)arg);
	}
	case RTC_EPOCH_SET:	/* Set the epoch.	*/
	{
		/* 
		 * There were no RTC clocks before 1900.
		 */
		if (arg < 1900)
			return -EINVAL;

		if (!capable(CAP_SYS_TIME))
			return -EACCES;

		epoch = arg;
		return 0;
	} 
	default:
		return -EINVAL;
	}
	return copy_to_user((void *)arg, &wtime, sizeof wtime) ? -EFAULT : 0;
}

/*
 *	We enforce only one user at a time here with the open/close.
 *	Also clear the previous interrupt data on an open, and clean
 *	up things on a close.
 */

/* We use rtc_lock to protect against concurrent opens. So the BKL is not
 * needed here. Or anywhere else in this driver. */
static int rtc_open(struct inode *inode, struct file *file)
{
	spin_lock_irq (&rtc_lock);

	if(rtc_status)
		goto out_busy;

	rtc_status = 1;

	spin_unlock_irq (&rtc_lock);
	return 0;

out_busy:
	spin_unlock_irq (&rtc_lock);
	return -EBUSY;
}

static int rtc_release(struct inode *inode, struct file *file)
{


	/* No need for locking -- nobody else can do anything until this rmw is
	 * committed, and no timer is running. */
	rtc_status = 0;
	return 0;
}

/*
 *	The various file operations we support.
 */

static struct file_operations rtc_fops = {
	owner:		THIS_MODULE,
	llseek:		no_llseek,
	ioctl:		rtc_ioctl,
	open:		rtc_open,
	release:	rtc_release,
};

#define RTC_MINOR   135

static struct miscdevice rtc_dev=
{
	RTC_MINOR,
	"rtc",
	&rtc_fops
};

int __init pcf_rtc_init(void)
{
      	int r;
	unsigned char ctr;
	r = misc_register(&rtc_dev);

	ctr = read_rtc(RTC_CONTROL);
	write_rtc(RTC_CONTROL,0x00 );

	read_rtcframe();
	CMOS_WRITE(RTC_STATUS, 0x00);
	CMOS_WRITE(RTC_CLKOUT, 0x80);
	/* RTC clock out, 32.768k */

	CMOS_WRITE(RTC_TIMERCTL, 0x00);
	CMOS_WRITE(RTC_TIMERCOUNTDOWN, 0x00);
	write_rtcframe();

	printk("PCF8563 RTC installed !!!\n");
	return 0;

}

void __exit pcf_rtc_exit (void)
{
	misc_deregister(&rtc_dev);
}

module_init(pcf_rtc_init);
module_exit(pcf_rtc_exit);
