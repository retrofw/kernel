#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <asm/jzsoc.h>
#include <asm/uaccess.h>

#define OPTR_BASE 0xB34100E0
#define REG_OPTR	REG32(OPTR_BASE)

#define OPT_WRITE_PROTECT       1
#define OPT_READ                2
#define OPT_WRITE               3
#define OPT_DUMP_REGS           4

#define __opt_wait_write()			\
	do {					\
		while (REG_OPTR&(0x1 << 1));	\
	} while(0)

#define __opt_wait_read()			\
	do {					\
		while (REG_OPTR&(0x1 << 0));	\
	} while(0)


static char driver_name[] = "opt_test";

struct opt_args {
	char mode;		/* 0->read 1->write */
	char section;		/* 128 bit has 8 sections */	
	unsigned short data;	/* data to read or write */
};


static int opt_write_protect(void)
{
	unsigned tmp = ((0x7 << 2) | (0x1 << 31) | (0x2));	
	tmp &= ~(0x1 << 5);
	REG_OPTR = tmp;
	__opt_wait_write();

	return 0;
}

/* before you write check the APB clock musb be 100Mhz*/
static int opt_test_write(char mode, char bit, unsigned short *data)

{	unsigned short val;
	unsigned int tmp = 0;

	if (mode == 1) {
		printk("you can't write chip space.\n");
		return 0;
	}
	val = *data;
	
	tmp |= (bit << 2); /* write section */
	tmp &= ~(0x1 << 5); /* write user area */
	
	tmp |= (val << 16);
	
	REG_OPTR = ((0x1 << 1) | tmp); /* write enable */
	__opt_wait_write();

	return 0;
}

static int opt_test_read(char mode, char bit, unsigned short *data)
{
	unsigned short val;
	unsigned int tmp = 0;


	tmp |= (bit << 2); /* read section*/
	/* read user space */
	if (mode == 0) {
		tmp &= ~(0x1 << 5); /* read user area */
	} else { 	
		tmp |= (0x1 << 5);  /* read chip serial No */
	}

	REG_OPTR = ((0x1 << 0) | tmp); /* READ enable */
	__opt_wait_read();

	val = (REG_OPTR & 0xffff0000) >> 16;

	if (data)
		*data = val;
	
	return val;
}

static void opt_dump_regs(void)
{
	int i;
	for (i = 0; i < 8; i++)
		printk("USER section (%d) DATA: 0x%04x\n", i, opt_test_read(0, i, 0));
	for (i = 0; i < 8; i++)
		printk("CHIP section (%d) DATA: 0x%04x\n", i, opt_test_read(1, i, 0));
}

static int opt_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int opt_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int rv = 0;
	struct opt_args my_args;

	rv = copy_from_user(&my_args, (struct opt_args *)arg, sizeof(struct opt_args));
	if (rv) {
		printk("copy from user space failed.\n");
		rv = -EFAULT;
		return rv;
	}
	
	switch(cmd) {
	case OPT_WRITE_PROTECT:
		opt_write_protect();
		break;
	case OPT_READ:
		opt_test_read(my_args.mode, my_args.section, &my_args.data);
		rv = copy_to_user((struct opt_args *)arg, &my_args, sizeof(struct opt_args));
		if (rv) {
			printk("copy to user space failed.\n");
			return -EFAULT;
		}
		break;
	case OPT_WRITE:
		opt_test_write(my_args.mode, my_args.section, &(my_args.data));
		break;

	case OPT_DUMP_REGS:
		opt_dump_regs();
		break;

	default:
		printk("unknown command.\n");
		break;
	}

	return rv;
}

static struct file_operations opt_fops = {
	.owner	=	THIS_MODULE,
	.open	=	opt_open,
	.ioctl	=	opt_ioctl,
};

static struct miscdevice misc_opt = {
	.minor	=	MISC_DYNAMIC_MINOR,
	.name	= 	(char *)driver_name,
	.fops	=	&opt_fops,
};

static int __init opt_test_init(void)
{
	int rv;

	rv = misc_register(&misc_opt);
	if (rv < 0) {
		printk("register misc device opt failed.\n");
		return rv;
	}

	printk("register misc device opt successed.\n");

	return 0;
}
static void __exit opt_test_exit(void)
{
	misc_deregister(&misc_opt);
	printk("deregister misc device opt.\n");
}

module_init(opt_test_init);
module_exit(opt_test_exit);

MODULE_LICENSE("Dual BSD/GPL");
