/*
 * Jz OnChip Real Time Clock interface for Linux
 *
 * NOTE: we need to wait rtc write ready before read or write RTC registers.
 *
 */

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
#include <linux/interrupt.h>
#include <linux/sched.h>

#include <linux/rtc.h>			/* get the user-level API */
#include <asm/system.h>
#include <asm/jzsoc.h>

#include "rtc_jz.h"


char sbin_rtc_alarm_handler_path[] = "/sbin/rtcalarm";
//call_usermodehelper(char *path, char **argv, char **envp, int wait)
//extern int call_usermodehelper(char *path, char **argv, char **envp);

extern spinlock_t rtc_lock;

static int rtc_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg);


static void get_rtc_time (struct rtc_time *rtc_tm);
static int  set_rtc_time (struct rtc_time *rtc_tm);
static void get_rtc_alm_time (struct rtc_time *alm_tm);
static int  set_rtc_alm_time (struct rtc_time *alm_tm);

static void set_rtc_irq_bit(int bit);
static void mask_rtc_irq_bit(int bit);

static unsigned int rtc_status = 0;
static unsigned int epoch = 1900;

static void get_rtc_time(struct rtc_time *rtc_tm)
{
	unsigned long lval;
	struct rtc_time ltm;

	spin_lock_irq(&rtc_lock);
	while ( !__rtc_write_ready() ) ;
	lval = REG_RTC_RSR;
	rtc_time_to_tm(lval, &ltm);
	if(rtc_valid_tm(&ltm) == 0) {
		/* is valid */
		rtc_tm->tm_sec = ltm.tm_sec;
		rtc_tm->tm_min = ltm.tm_min;
		rtc_tm->tm_hour = ltm.tm_hour;
		rtc_tm->tm_mday = ltm.tm_mday;
		rtc_tm->tm_wday = ltm.tm_wday;
		rtc_tm->tm_mon = ltm.tm_mon;
		rtc_tm->tm_year = ltm.tm_year;
	} else {
		printk("invlaid data / time!\n");
	}
	spin_unlock_irq(&rtc_lock);
}

static int set_rtc_time(struct rtc_time *rtc_tm)
{
	unsigned long lval;

	rtc_tm_to_time(rtc_tm, &lval);

	spin_lock_irq(&rtc_lock);
	while ( !__rtc_write_ready() ) ;
	REG_RTC_RSR = lval;

	spin_unlock_irq(&rtc_lock);

	return 0;

}

static void get_rtc_alm_time(struct rtc_time *alm_tm)
{ 
	unsigned long lval;
	struct rtc_time altm;

	spin_lock_irq(&rtc_lock);
	while ( !__rtc_write_ready() ) ;
	lval = REG_RTC_RSAR;
	rtc_time_to_tm(lval, &altm);
	if(rtc_valid_tm(&altm) == 0) {
		/* is valid */
		alm_tm->tm_sec = altm.tm_sec;
		alm_tm->tm_min = altm.tm_min;
		alm_tm->tm_hour = altm.tm_hour;
		alm_tm->tm_mday = altm.tm_mday;
		alm_tm->tm_wday = altm.tm_wday;
		alm_tm->tm_mon = altm.tm_mon;
		alm_tm->tm_year = altm.tm_year;
	} else {
		printk("invlaid data / time in Line:%d!\n",__LINE__);
	}
	spin_unlock_irq(&rtc_lock);
}

static int set_rtc_alm_time(struct rtc_time *alm_tm)
{
	unsigned long lval;

	rtc_tm_to_time(alm_tm, &lval);

	spin_lock_irq(&rtc_lock);
	while ( !__rtc_write_ready() ) ;
	REG_RTC_RSAR = lval;

	while ( !__rtc_write_ready() ) ; /* set alarm function */
	if ( !((REG_RTC_RCR>>2) & 0x1) ) {
		while ( !__rtc_write_ready() ) ;
		__rtc_enable_alarm();
	}

	while ( !__rtc_write_ready() ) ;
	if ( !(REG_RTC_RCR & RTC_RCR_AIE) ) { /* Enable alarm irq */
		__rtc_enable_alarm_irq();
	}

	spin_unlock_irq(&rtc_lock);

	return 0;
}

static void get_rtc_wakeup_alarm(struct rtc_wkalrm *wkalm)
{
	int enabled, pending;

	get_rtc_alm_time(&wkalm->time);

	spin_lock_irq(&rtc_lock);
	while ( !__rtc_write_ready() ) ;
	enabled = (REG_RTC_HWCR & 0x1);
	pending = 0;
	if ( enabled ) {
		if ( (u32)REG_RTC_RSAR > (u32)REG_RTC_RSR ) /* 32bit val */
			pending = 1;
	}

	wkalm->enabled = enabled;
	wkalm->pending = pending;
	spin_unlock_irq(&rtc_lock);
}

static int set_rtc_wakeup_alarm(struct rtc_wkalrm *wkalm)
{
	int enabled;
	//int pending;

	enabled = wkalm->enabled;
	//pending = wkalm->pending; /* Fix me, what's pending mean??? */

	while ( !__rtc_write_ready() ) ; /* set wakeup alarm enable */
	if ( enabled != (REG_RTC_HWCR & 0x1) ) {
		while ( !__rtc_write_ready() ) ;
		REG_RTC_HWCR = (REG_RTC_HWCR & ~0x1) | enabled;
	}
	while ( !__rtc_write_ready() ) ; /* set alarm function */
	if ( enabled != ((REG_RTC_RCR>>2) & 0x1) ) {
		while ( !__rtc_write_ready() ) ;
		REG_RTC_RCR = (REG_RTC_RCR & ~(1<<2)) | (enabled<<2);
	}

	if ( !enabled )		/* if disabled wkalrm, rturn.  */
	{
		return 0;
	}

	while ( !__rtc_write_ready() ) ;
	if ( !(REG_RTC_RCR & RTC_RCR_AIE) ) { /* Enable alarm irq */
		__rtc_enable_alarm_irq();
	}

	set_rtc_alm_time(&wkalm->time);

	return 0;
}


static void set_rtc_irq_bit( int bit )
{
	spin_lock_irq(&rtc_lock);

	while ( !__rtc_write_ready() ) ;
	REG_RTC_RCR |= (1<<bit);

	spin_unlock_irq(&rtc_lock);
}

static void mask_rtc_irq_bit( int bit )
{
	spin_lock_irq(&rtc_lock);

	while ( !__rtc_write_ready() ) ;
	REG_RTC_RCR &= ~(1<<bit);


	spin_unlock_irq(&rtc_lock);
}

static int rtc_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     unsigned long arg)
{
	struct rtc_time wtime; 

	switch (cmd) {
	case RTC_AIE_OFF:	/* Mask alarm int. enab. bit	*/
	{
		mask_rtc_irq_bit(RTC_AIE);
		return 0;
	}
	case RTC_AIE_ON:	/* Allow alarm interrupts.	*/
	{
		__rtc_clear_alarm_flag();
		set_rtc_irq_bit(RTC_AIE);
		return 0;
	}
	case RTC_1HZIE_OFF:	/* Mask 1Hz int. enab. bit	*/
	{
		mask_rtc_irq_bit(RTC_1HZIE);
		return 0;
	}
	case RTC_1HZIE_ON:	/* Allow 1Hz interrupts.	*/
	{
		__rtc_clear_1Hz_flag();
		set_rtc_irq_bit(RTC_1HZIE);
		return 0;
	}
	case RTC_ALM_OFF:	/* Disable rtc function, this may not be used any time.*/
	{
		mask_rtc_irq_bit(RTC_ALM_EN);
		return 0;
	}
	case RTC_ALM_ON:	/* Enable rtc function, this may not be used any time.*/
	{
		set_rtc_irq_bit(RTC_ALM_EN);
		return 0;
	}
	case RTC_DISABLED:	/* Disable rtc function, this may not be used any time.*/
	{
		mask_rtc_irq_bit(RTC_EN);
		return 0;
	}
	case RTC_ENABLED:	/* Enable rtc function, this may not be used any time.*/
	{
		set_rtc_irq_bit(RTC_EN);
		return 0;
	}

	case RTC_ALM_READ:	/* Read the present alarm time */
		/*
		 * This returns a struct rtc_time. Reading >= 0xc0
		 * means "don't care" or "match all". Only the tm_hour,
		 * tm_min, and tm_sec values are filled in.
		 */
		
		get_rtc_alm_time(&wtime);
		return copy_to_user((void *)arg, &wtime, sizeof wtime) ? -EFAULT : 0;

	case RTC_ALM_SET:	/* Store a time into the alarm */
	{
		struct rtc_time alm_tm;

		if (copy_from_user(&alm_tm, (struct rtc_time*)arg,
				   sizeof(struct rtc_time)))
			return -EFAULT;
		if(rtc_valid_tm(&alm_tm) != 0) {
			printk("invalid time set in Line:%d! \n",__LINE__);
			return -EFAULT;
		}
		
		return set_rtc_alm_time(&alm_tm);
	}
	case RTC_RD_TIME:	/* Read the time/date from RTC	*/
		get_rtc_time(&wtime);
		return copy_to_user((void *)arg, &wtime, sizeof wtime) ? -EFAULT : 0;
	case RTC_SET_TIME:	/* Set the RTC */
	{
		struct rtc_time rtc_tm;

		if (!capable(CAP_SYS_TIME))
			return -EACCES;

		if (copy_from_user(&rtc_tm, (struct rtc_time*)arg,
				   sizeof(struct rtc_time)))
			return -EFAULT;
		if(rtc_valid_tm(&rtc_tm) != 0) {
			printk("invalid time set in Line:%d! \n",__LINE__);
			return -EFAULT;
		}
	      
		return set_rtc_time(&rtc_tm);
	}
	case RTC_EPOCH_READ:	/* Read the epoch.	*/
		return put_user (epoch, (unsigned long *)arg);
	case RTC_EPOCH_SET:	/* Set the epoch.	*/
		/* 
		 * There were no RTC clocks before 1900.
		 */
		if (arg < 1900)
			return -EINVAL;

		if (!capable(CAP_SYS_TIME))
			return -EACCES;

		epoch = arg;
		return 0;
	case RTC_WKALM_SET:	/* Wake alarm set.	*/
	{
		struct rtc_wkalrm wkalrm;

		if (copy_from_user(&wkalrm, (struct rtc_wkalrm*)arg,
				   sizeof(struct rtc_wkalrm)))
			return -EFAULT;
		return set_rtc_wakeup_alarm(&wkalrm);
	}
	case RTC_WKALM_RD:	/* Wake alarm read.	*/
	{
		struct rtc_wkalrm wkalrm;
		get_rtc_wakeup_alarm(&wkalrm);
		return copy_to_user((void *)arg, &wkalrm, sizeof(struct rtc_wkalrm)) ? -EFAULT : 0;
	}
	/* set power down: shut down the machine. */
	case RTC_POWER_DOWN:	/* enter HIBERNATE mode */
		dprintk("Power down. Bye....\n");
		while ( !__rtc_write_ready() ) ;
		REG_RTC_HCR = 0x1;
		return 0;
#ifdef DEBUG
	case RTC_PRINT_REG:	/* Print RTC registers */
		print_rtc_registers();
		return 0;
#endif
	default:
		return -EINVAL;
	}
	return 0;
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

	rtc_status = 0;
	/* No need for locking -- nobody else can do anything until this rmw is
	 * committed, and no timer is running. */
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


static void run_sbin_rtc_alarm( void )
{
	int i;
	char *argv[2], *envp[3];

	if (!sbin_rtc_alarm_handler_path[0])
		return;

	print_dbg(": sbin_rtc_alarm_handler_path=%s\n", sbin_rtc_alarm_handler_path);

	i = 0;
	argv[i++] = sbin_rtc_alarm_handler_path;
	argv[i] = 0;

	/* minimal command environment */
	i = 0;
	envp[i++] = "HOME=/";
	envp[i++] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
	
	/* other stuff we want to pass to /sbin/hotplug */

	envp[i] = 0;

	call_usermodehelper (argv [0], argv, envp, 0);
}

static void rtc_alarm_task_handler(struct work_struct *work)
{
	run_sbin_rtc_alarm();
}

static struct work_struct rtc_alarm_task;

static irqreturn_t jz_rtc_interrupt(int irq, void *dev_id)
{
	REG_RTC_HCR = 0x0;
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	spin_lock_irq(&rtc_lock);

	if ( __rtc_get_1Hz_flag() ) {
		while ( !__rtc_write_ready() ) ;
		__rtc_clear_1Hz_flag();
		dprintk("RTC 1Hz interrupt occur.\n");
	}

	if ( __rtc_get_alarm_flag() ) {	/* rtc alarm interrupt */
		while ( !__rtc_write_ready() ) ;
		__rtc_clear_alarm_flag();
		dprintk("RTC alarm interrupt occur.\n");
		//schedule_task( &rtc_alarm_task );
		schedule_work( &rtc_alarm_task );
	}
	spin_unlock_irq(&rtc_lock);

	return IRQ_HANDLED;
}


#define RTC_MINOR   135

static struct miscdevice rtc_dev=
{
	RTC_MINOR,
	"rtc",
	&rtc_fops
};

/* The divider is decided by the RTC clock frequency. */
#define RTC_FREQ_DIVIDER	(32768 - 1)
#define ms2clycle(x)  (((x) * RTC_FREQ_DIVIDER) / 1000)

#define RTC_CFG (((unsigned int)('R') << 24) | ((unsigned int)('T') << 16) | ((unsigned int)('C') << 8))
#define CAL_RTC_CFG(x)				\
  ({						\
    unsigned int x1,x2,x3;			\
    x1 = ((x) >> 24) & 0xff;			\
    x2 = ((x) >> 16) & 0xff;			\
    x3 = ((x) >> 8) & 0xff;			\
  ((x & (~0xff)) | (x1 + x2 + x3));		\
  })
#define SET_RTC_REG(reg,x)				\
  do{							\
    unsigned int rcr;					\
    do{							\
      rcr = reg;					\
    }while(rcr != reg);					\
    rcr |= (x);						\
    while ( !__rtc_write_ready());			\
    reg = rcr;						\
  }while(0);

#define CLR_RTC_REG(reg,x)				\
  do{							\
    unsigned int rcr;					\
    do{							\
      rcr = reg;					\
    }while(rcr != reg);					\
    rcr &= ~(x);					\
    while ( !__rtc_write_ready());			\
    reg = rcr;						\
  }while(0);

#define OUT_RTC_REG(reg,x)				\
  do{							\
    while ( !__rtc_write_ready());			\
    reg = x;						\
  }while(0);

#define IN_RTC_REG(reg)					\
  ({							\
    unsigned int dat;					\
    do{							\
      dat = reg;					\
    }while(reg != dat);					\
    dat;						\
  })


 /* Default time for the first-time power on */
static struct rtc_time default_tm = {
	.tm_year = (2009 - 1900), // year 2009
	.tm_mon = (10 - 1),       // month 10
	.tm_mday = 1,             // day 1
	.tm_hour = 12,
	.tm_min = 0,
	.tm_sec = 0
}; 
static void rtc_first_power_on()
{
	unsigned int rcr,cfc,hspr,rgr_1hz;
	/*
	 * When we are powered on for the first time, init the rtc and reset time.
	 *
	 * For other situations, we remain the rtc status unchanged.
	 */
	 
	__cpm_select_rtcclk_rtc();

	//unsigned int ppr = IN_RTC_REG(REG_RTC_HWRSR);   
	cfc = 0x12345678;//CAL_RTC_CFG(RTC_CFG);
	hspr = IN_RTC_REG(REG_RTC_HSPR);
	rgr_1hz  = IN_RTC_REG(REG_RTC_RGR) & RTC_RGR_NC1HZ_MASK;
	
	if((hspr != cfc) || (rgr_1hz != RTC_FREQ_DIVIDER))
	{
	//if ((ppr >> RTC_HWRSR_PPR) & 0x1) {
		/* We are powered on for the first time !!! */

		printk("jz4750-rtc: rtc status reset by power-on\n");

		/* init rtc status */
		
		rcr = IN_RTC_REG(REG_RTC_RCR);
		rcr &= ~(RTC_RCR_1HZ | RTC_RCR_1HZIE | RTC_RCR_AF | RTC_RCR_AE | RTC_RCR_AIE);
		

		/* Set 32768 rtc clocks per seconds */
		OUT_RTC_REG(REG_RTC_RGR,RTC_FREQ_DIVIDER);

		/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */

		OUT_RTC_REG(REG_RTC_HWFCR,ms2clycle(100) << RTC_HWFCR_BIT);

		//REG_RTC_HWFCR = (100 << RTC_HWFCR_BIT);
		//while ( !__rtc_write_ready());
	
		/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
		//REG_RTC_HRCR = (60 << RTC_HRCR_BIT);
		//while ( !__rtc_write_ready());

		OUT_RTC_REG(REG_RTC_HRCR,ms2clycle(60) <<  RTC_HRCR_BIT);
                /* Reset to the default time */
		set_rtc_time( &default_tm);
		/* start rtc */
		rcr |= RTC_RCR_RTCE;
		OUT_RTC_REG(REG_RTC_RCR,rcr);
		OUT_RTC_REG(REG_RTC_HSPR,cfc);
		/* select external 32K crystal as RTC clock */
	}
}
int __init Jz_rtc_init(void)
{

	printk("jz4750-rtc: Jz_rtc_init\n");
	rtc_first_power_on();
	INIT_WORK(&rtc_alarm_task, rtc_alarm_task_handler);

	/* Enabled rtc function, enable rtc alarm function */
	while ( !__rtc_write_ready() ) ; /* need we wait for WRDY??? */
	if ( !(REG_RTC_RCR & RTC_RCR_RTCE) || !(REG_RTC_RCR &RTC_RCR_AE) ) {
		REG_RTC_RCR |= RTC_RCR_AE | RTC_RCR_RTCE;
	}
	/* clear irq flags */
	__rtc_clear_1Hz_flag();
	/* In a alarm reset, we expect a alarm interrupt. 
	 * We can do something in the interrupt handler.
	 * So, do not clear alarm flag.
	 */
/*	__rtc_clear_alarm_flag(); */

	if (request_irq(IRQ_RTC, jz_rtc_interrupt, 0, "rtc", NULL) < 0)
		return -EBUSY;

	misc_register(&rtc_dev);

	printk("JzSOC onchip RTC installed !!!\n");
	return 0;

}

void __exit Jz_rtc_exit (void)
{
	misc_deregister(&rtc_dev);
	free_irq (IRQ_RTC, NULL);
}

module_init(Jz_rtc_init);
module_exit(Jz_rtc_exit);

