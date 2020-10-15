/* ****************************************************************
 * @ func   : L009 Game Keypad Driver
 * @ author : maddrone@gmail.com
 * TODO:change this file name in git
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
//#include <linux/pm.h>
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



#define L009_KEY_UP     0
#define L009_KEY_DOWN   1
#define L009_KEY_LEFT   2
#define L009_KEY_RIGHT  3
#define L009_KEY_A      4
#define L009_KEY_B      5
#define L009_KEY_X      6
#define L009_KEY_Y      7
#define L009_KEY_L      8
#define L009_KEY_R      9
#define L009_KEY_START    10
#define L009_KEY_SELECT   11
#define L009_KEY_HOLD     12
#define L009_KEY_POWER    13

static unsigned int key_value;

unsigned int jz_button[] =
{
	(UMIDO_KEY_UP    ),	// UP        0
	(UMIDO_KEY_DOWN  ),   // DOWN      1
	(UMIDO_KEY_LEFT  ),   // LEFT      2
	(UMIDO_KEY_RIGHT ),   // RIGHT     3
	(UMIDO_KEY_A     ),  // A         4
	(UMIDO_KEY_B     ),  // B         5
	(UMIDO_KEY_Y     ),  // Y         6
	(UMIDO_KEY_X     ),  // X         7
	(UMIDO_KEY_UP_L1),   // up L         8
	(UMIDO_KEY_UP_L2),  // up L2         9
	(UMIDO_KEY_UP_R1),  // up R          10
	(UMIDO_KEY_UP_R2),	// up R2		11
	(UMIDO_KEY_DOWN_L1),	// down L		12
	(UMIDO_KEY_DOWN_L2),	// down L2		13
	(UMIDO_KEY_DOWN_R1),	// down R		14
	(UMIDO_KEY_DOWN_R2),	// down R2		15
	(UMIDO_KEY_START ),  // START		16
	(UMIDO_KEY_SELECT),   // SELECT		17
	//(32 * 3 + 18),  // HOLD      12
	//(125)           // POWER     13
};

static int key_open(struct inode *inode, struct file *filp)
{
	printk (KERN_ALERT "key_open\n");
	key_value = 0;
	return 0;
}

static int key_release(struct inode *inode, struct file *filp)
{
	printk(KERN_ALERT "key_release\n");
	return 0;
}

extern unsigned int l009_ts_read();
extern unsigned int jz_read_ts_data();

unsigned int check_num_0 = 0;

static ssize_t key_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int i;
	unsigned int val;

	key_value = 0;
	for(i=0; i<ARRAY_SIZE(jz_button); i++)
	{
#if H350_V0
		if(i == 8 || i == 10)
		{

			if((__gpio_get_pin(jz_button[i])))  key_value |= (1 << i);
		}
		else
		{
			if(!(__gpio_get_pin(jz_button[i])))  key_value |= (1 << i);
		}
#else
		if(i == 16)
		{

			if((__gpio_get_pin(jz_button[i])))  key_value |= (1 << i);
		}
		else
		{
			if(!(__gpio_get_pin(jz_button[i])))  key_value |= (1 << i);
		}
//		if(key_value != 0 && (key_value >> i == 1))
//			printk("key value is 0x%x,i is %d\n",key_value,i);
#endif
	}

#if 0 //allen del
        if(l009_gsensor_flag)
        {
          key_value |= l009_gsensor_read();
        }
#endif

	//key_value |= jz_read_ts_data();

#ifdef H350_HOLD_PIN
	if (key_value != 0){
		if (!__gpio_get_pin(H350_HOLD_PIN)){
			key_value = 0;
		}
	}
#endif

	copy_to_user(buf, &key_value, sizeof(int));

	return sizeof(int);
}

#define 	KEYPAD_MAJOR 		252
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

	ret = misc_register(&keypad_device);
	if(ret<0){
		printk("kernel : keypad register failed!\n");
	}

	for(i=0; i<ARRAY_SIZE(jz_button); i++)
	{
#if H350_V0
          if(i != 8 && i != 10)
          {
		__gpio_as_input(jz_button[i]);
		__gpio_enable_pull(jz_button[i]);
          }else{
		__gpio_as_input(jz_button[i]);
		__gpio_disable_pull(jz_button[i]);
	  }
#else
	  if(i != 16)
	  {
		  __gpio_as_input(jz_button[i]);
		  __gpio_disable_pull(jz_button[i]);
	  }else{
		  __gpio_as_input(jz_button[i]);
		  __gpio_disable_pull(jz_button[i]);
	  }

#endif
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


