/* ****************************************************************
 * @ func   : L009 Game Keypad Driver
 * @ author : maddrone@gmail.com
 * ****************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
//#include <linux/pm_legacy.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>

static unsigned int key_value;

unsigned int umido_button[] =
{
	UMIDO_KEY_UP,
    UMIDO_KEY_DOWN,
    UMIDO_KEY_LEFT,
    UMIDO_KEY_RIGHT,
    UMIDO_KEY_A,
    UMIDO_KEY_B,
    UMIDO_KEY_X,
    UMIDO_KEY_Y,
    UMIDO_KEY_START,
    UMIDO_KEY_SELECT,
    UMIDO_KEY_L,
    UMIDO_KEY_R,
    // GPIO_POWER_ON,
};

#define TOTAL_BUTTON_NUM sizeof(umido_button)/sizeof(unsigned int)

static int key_open(struct inode *inode, struct file *filp)
{
	//is_powerkey_coming_now = true;
	printk (KERN_ALERT "key_open\n");
	key_value = 0;
	return 0;
}

static int key_release(struct inode *inode, struct file *filp)
{
	//is_powerkey_coming_now = false;
	printk(KERN_ALERT "key_release\n");
	return 0;
}

extern unsigned int jz_read_key_handle();

// extern void fb_resize_start();
static int other_key_value = 0;
void umido_gamepad_set_key_value(int value,char joystick)
{

    //printk("1other_key_value is 0x%x joystick is 0x%x %x %x\n", other_key_value,joystick,value,value<<16);
    if (joystick == 1) {
	other_key_value = (other_key_value & 0xffff0000)|value;
    }
    else if (joystick == 2) {
	other_key_value = (other_key_value & 0x0000ffff)|(value << 16);
    }
    else
    {
	other_key_value = (other_key_value & 0xffff0000)|value;
    }
    //printk("2other_key_value is 0x%x joystick is 0x%x \n", other_key_value,joystick);
}
EXPORT_SYMBOL_GPL(umido_gamepad_set_key_value);

static ssize_t key_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int i = 0;
	key_value = 0;

	for(i = 0; i < TOTAL_BUTTON_NUM; i++)
	{

		if(i == 8 || i == 9)
		{
			if(__gpio_get_pin(umido_button[i]) != 0)
				key_value |= 1 << i;
		}
		else
		{
			if(__gpio_get_pin(umido_button[i]) == 0)
			{
				//printk("%d \n",i);
				key_value |= 1 << i;
			}
		}
	}

#if 0 //adc key

	key_value |= jz_read_key_handle();  //medive add
	key_value |= other_key_value;

#endif


#if 0 //common 2.4G
	extern unsigned int common_2G4_value;
	int key_2G4 = common_2G4_value;

	key_value |= (key_2G4 & 0xffffffff);
#endif
#if 1 //allen add power

	if(__gpio_get_pin(GPIO_POWER_ON) == 0)
		key_value = 0X300;
#endif

	//if(key_value)
	//	printk("key value is 0x%x\n",key_value);

	//fb_resize_start();
  	copy_to_user(buf, &key_value, sizeof(int));
  	return sizeof(int);
}

#define 	KEYPAD_DEVICE_NAME	"keypad"

static struct file_operations keypad_fops = {
    owner:              THIS_MODULE,
    open:               key_open,
    read:               key_read,
    release:            key_release,
};

static struct miscdevice keypad_device = {
	.minor      = KEYPAD_MAJOR,
	.name       = KEYPAD_DEVICE_NAME,
	.fops       = &keypad_fops,
};


static int __init keypad_init(void)
{
	int i,ret;

	printk("umido gamepade ==================== init \n");

	ret = misc_register(&keypad_device);
	if(ret<0)
		printk("kernel : keypad register failed!\n");

       for(i = 0; i < TOTAL_BUTTON_NUM; i++)
       {
	       if (i == 8 || i == 9){
		       __gpio_as_func0(umido_button[i]);
		       __gpio_as_input(umido_button[i]);
		       __gpio_disable_pull(umido_button[i]);
	       }
		   else
	       {
		       __gpio_as_func0(umido_button[i]);
		       __gpio_as_input(umido_button[i]);
		       __gpio_enable_pull(umido_button[i]);
	       }
       }

	return 0;
}

static void __exit keypad_cleanup(void)
{
	return ;
}

module_init(keypad_init);
module_exit(keypad_cleanup);

MODULE_AUTHOR("maddrone");
MODULE_DESCRIPTION("L009 keypad driver");
MODULE_LICENSE("GPL");


