/*
 * linux/drivers/char/jz_wdt.c
 *
 * Watchdog driver for the Ingenic JzSOC
 *
 * Author: Wei Jianli <jlwei@ingenic.cn>
 *
 * 2005 (c) Ingenic Semiconductor. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
//#include <linux/config.h>
#include <linux/autoconf.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/jzsoc.h>

#define TIMER_MARGIN	60		/* (secs) Default is 1 minute */

static unsigned int timer_margin = TIMER_MARGIN; /* in seconds */
static unsigned int timer_rate;
static unsigned int pre_margin;
static unsigned long jz_wdt_users = 0;

#ifdef MODULE
MODULE_PARM(timer_margin, "i");
#endif

static void
jz_wdt_ping(void)
{
	printk("jz_wdt_ping\n");
	/* reload counter with (new) margin */
#ifdef CONFIG_SOC_JZ4730
	pre_margin = 0xffffffff - timer_rate * timer_margin;
	__wdt_set_count(pre_margin);
#endif
#ifdef CONFIG_SOC_JZ4740
	pre_margin = timer_rate * timer_margin;
	__wdt_set_count(0);
	__wdt_set_data(pre_margin);
#endif
}

/*
 *	Allow only one person to hold it open
 */

static int
jz_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, &jz_wdt_users))
		return -EBUSY;

	printk("jz_wdt_open\n");
#ifdef CONFIG_SOC_JZ4730
	if (REG_CPM_OCR & CPM_OCR_EXT_RTC_CLK)
		timer_rate = 32768;
	else
		timer_rate = JZ_EXTAL/128;
#endif

#ifdef CONFIG_SOC_JZ4740
	/* Initialize the wdt clocks */
	__wdt_select_rtcclk();
	__wdt_select_clk_div1024();
	__tcu_start_wdt_clock();
	timer_rate = 32; /* 32768 / 1024 */
#endif

	jz_wdt_ping();
	__wdt_start();

	return 0;
}

static int
jz_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *      Shut off the timer.
	 *      Lock it in if it's a module and we defined ...NOWAYOUT
	 */
	jz_wdt_ping();
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	/* SW can't stop wdt once it was started */
#endif
	jz_wdt_users = 0;
	return 0;
}

static ssize_t
jz_wdt_write(struct file *file, const char *data, size_t len, loff_t * ppos)
{
	/*  Can't seek (pwrite) on this device  */
	if (ppos != &file->f_pos)
		return -ESPIPE;

	printk("jz_wdt_write\n");

	/* Refresh counter */
	if (len) {
		jz_wdt_ping();
		return 1;
	}
	return 0;
}

static int
jz_wdt_ioctl(struct inode *inode, struct file *file,
	       unsigned int cmd, unsigned long arg)
{
	int new_margin;
	static struct watchdog_info ident = {
		.identity		= "JzSOC Watchdog",
		.options		= WDIOF_SETTIMEOUT,
		.firmware_version	= 0,
	};

	switch (cmd) {
	default:
		return -ENOIOCTLCMD;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info *) arg, &ident,
				    sizeof (ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int *) arg);
	case WDIOC_GETBOOTSTATUS:
#ifdef CONFIG_SOC_JZ4730
		return put_user(REG_CPM_RSTR, (int *) arg);
#endif
#ifdef CONFIG_SOC_JZ4740
		return put_user(REG_RTC_HWRSR, (int *) arg);
#endif
	case WDIOC_KEEPALIVE:
		jz_wdt_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int *) arg))
			return -EFAULT;
		if (new_margin < 1)
			return -EINVAL;
		timer_margin = new_margin;
		jz_wdt_ping();
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(timer_margin, (int *) arg);
	}
}

static struct file_operations jz_wdt_fops = {
	.owner		= THIS_MODULE,
	.write		= jz_wdt_write,
	.ioctl		= jz_wdt_ioctl,
	.open		= jz_wdt_open,
	.release	= jz_wdt_release,
};

static struct miscdevice jz_wdt_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "jz_wdt",
	.fops	= &jz_wdt_fops
};

static int __init
jz_wdt_init(void)
{
	int ret;

	ret = misc_register(&jz_wdt_miscdev);

	if (ret)
		return ret;

	printk("JzSOC Watchdog Timer: timer margin %d sec\n", timer_margin);

	return 0;
}

static void __exit
jz_wdt_exit(void)
{
	misc_deregister(&jz_wdt_miscdev);
}

module_init(jz_wdt_init);
module_exit(jz_wdt_exit);

MODULE_AUTHOR("Wei Jianli");
MODULE_LICENSE("GPL");
