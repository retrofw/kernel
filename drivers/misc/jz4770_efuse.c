#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fs.h>

#include <asm/jzsoc.h>
#include <asm/uaccess.h>

#define EFUSE_WRITE_LOW_BIT		1
#define EFUSE_WRITE_HIGH_BIT		2
#define EFUSE_WRITE_FULL_BIT		3
#define EFUSE_LOW_DISABLE_PROG		4
#define EFUSE_HIGH_DISABLE_PROG		5
#define EFUSE_DUMP_REGS			6

#define GPIO_EFUSE_VDDQ_POWER		(16)

static unsigned int data[8];

static void efuse_dump_regs(void)
{
	printk("REG_EFUSE_DATA0	= %08x\n", REG_EFUSE_DATA0);
	printk("REG_EFUSE_DATA1	= %08x\n", REG_EFUSE_DATA1);
	printk("REG_EFUSE_DATA2	= %08x\n", REG_EFUSE_DATA2);
	printk("REG_EFUSE_DATA3	= %08x\n", REG_EFUSE_DATA3);
	printk("REG_EFUSE_DATA4	= %08x\n", REG_EFUSE_DATA4);
	printk("REG_EFUSE_DATA5	= %08x\n", REG_EFUSE_DATA5);
	printk("REG_EFUSE_DATA6	= %08x\n", REG_EFUSE_DATA6);
	printk("REG_EFUSE_DATA7	= %08x\n", REG_EFUSE_DATA7);
}

static char dev_name[] = "jz4770_efuse";


static int efuse_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int rv = 0;
	int i;
	unsigned int *tmp = data;

	if (cmd == EFUSE_WRITE_LOW_BIT || cmd == EFUSE_WRITE_HIGH_BIT ||
		cmd == EFUSE_WRITE_FULL_BIT) {
		rv = copy_from_user(tmp, (unsigned int *)arg, 32*8);
		if (rv) {
			printk("copy from user space failed.\n");
			rv = -EFAULT;

			return rv;
		}
	}
	
	switch(cmd) {
	case EFUSE_WRITE_LOW_BIT:
		REG_EFUSE_DATA0 = tmp[0];
		REG_EFUSE_DATA1 = tmp[1];
		REG_EFUSE_DATA2 = tmp[2];
		REG_EFUSE_DATA3 = tmp[3];

		__gpio_as_output0(GPIO_EFUSE_VDDQ_POWER);
		efuse_low_bit_enable(); 
		efuse_wait_write_low_ready();
		__gpio_as_output1(GPIO_EFUSE_VDDQ_POWER);
		
		break;
	case EFUSE_WRITE_HIGH_BIT:
		REG_EFUSE_DATA4 = tmp[4];
		REG_EFUSE_DATA5 = tmp[5];
		REG_EFUSE_DATA6 = tmp[6];
		REG_EFUSE_DATA7 = tmp[7];

		__gpio_as_output0(GPIO_EFUSE_VDDQ_POWER);
		efuse_high_bit_enable(); 
		efuse_wait_write_high_ready();
		__gpio_as_output1(GPIO_EFUSE_VDDQ_POWER);

		break;
	case EFUSE_WRITE_FULL_BIT:
		REG_EFUSE_DATA0 = tmp[0];
		REG_EFUSE_DATA1 = tmp[1];
		REG_EFUSE_DATA2 = tmp[2];
		REG_EFUSE_DATA3 = tmp[3];
		REG_EFUSE_DATA4 = tmp[4];
		REG_EFUSE_DATA5 = tmp[5];
		REG_EFUSE_DATA6 = tmp[6];
		REG_EFUSE_DATA7 = tmp[7];

		__gpio_as_output0(GPIO_EFUSE_VDDQ_POWER);
		efuse_full_bit_enable();
		efuse_wait_write_full_ready();
		__gpio_as_output1(GPIO_EFUSE_VDDQ_POWER);
	
		break;
	case EFUSE_LOW_DISABLE_PROG:
		printk("disable low bit prog.\n");
		efuse_low_disable_prog(); 

		break;
	case EFUSE_HIGH_DISABLE_PROG:
		printk("disable high bit prog.\n");
		efuse_low_disable_prog(); 

		break;
	case EFUSE_DUMP_REGS:
		efuse_dump_regs();

		break;
	default:
		printk("unkonwn efuse command.\n");
		break;
	}

	return rv;
}

static struct file_operations efuse_fops = {
        .owner  =       THIS_MODULE,
        .ioctl  =       efuse_ioctl,
};

static struct miscdevice misc_efuse = {
	.minor	=	MISC_DYNAMIC_MINOR,
	.name	=	(char *)dev_name,
	.fops	=	&efuse_fops,
};


static __init int efuse_init(void)
{
	int rv = 0;
	rv = misc_register(&misc_efuse);
	if (rv < 0) {
		printk("register misc device efuse failed.\n");
		return rv;
	}

	printk("register misc device efuse successed.\n");
	
	return rv;
}

static __exit void efuse_exit(void)
{
	misc_deregister(&misc_efuse);
	printk("deregister misc device efuse.\n");
}

module_init(efuse_init);
module_exit(efuse_exit);
MODULE_LICENSE("GPL");
