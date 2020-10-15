#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <asm/atomic.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/poll.h>

#include <asm/jzsoc.h>

/* ioctl command */
#define	WIEGAND_PULSE_WIDTH	0x40
#define	WIEGAND_PULSE_INTERVAL	0x50
#define WIEGAND_FORMAT		0x60
#define WIEGAND_READ		0x70
#define WIEGAND_STATUS		0x80

#define JZ_WIEGAND_MAJOR 0	      /* 0: dyna alloc */
static int jz_wiegand_major = JZ_WIEGAND_MAJOR;

#define MAX_WIEGAND_BITS	64
static int wiegand_format = 26; /* default to 26bits */

#define JZ_WIEGAND_TCU_CHANNEL	3

static void jz_wiegand_tcu_init(void) {
	__tcu_set_pwm_output_shutdown_abrupt(JZ_WIEGAND_TCU_CHANNEL);

	/* we do not need irq */
	__tcu_mask_full_match_irq(JZ_WIEGAND_TCU_CHANNEL);
	__tcu_mask_half_match_irq(JZ_WIEGAND_TCU_CHANNEL);

	__tcu_stop_counter(JZ_WIEGAND_TCU_CHANNEL);

	__tcu_select_extalclk(JZ_WIEGAND_TCU_CHANNEL);

	/* abount 1ms per count */
	__tcu_select_clk_div1024(JZ_WIEGAND_TCU_CHANNEL);

	REG_TCU_TDFR(JZ_WIEGAND_TCU_CHANNEL) = 0xFFFF; /* actually, we do not want the count to reset to 0, so use the MAX value */
	REG_TCU_TDHR(JZ_WIEGAND_TCU_CHANNEL) = 0x7FFF;
	REG_TCU_TCNT(JZ_WIEGAND_TCU_CHANNEL) = 0;

}

static void jz_wiegand_start_tcu_counter(void) {
	REG_TCU_TCNT(JZ_WIEGAND_TCU_CHANNEL) = 0;
	__tcu_clear_full_match_flag(JZ_WIEGAND_TCU_CHANNEL);
	__tcu_start_counter(JZ_WIEGAND_TCU_CHANNEL);
}

static void jz_wiegand_stop_tcu_counter(void) {
	__tcu_stop_counter(JZ_WIEGAND_TCU_CHANNEL);
}

static unsigned int jz_wiegnad_get_tcu_counter(void) {
	return __tcu_get_count(JZ_WIEGAND_TCU_CHANNEL);
}

struct wiegand_data {
	struct list_head list;
	unsigned long data[2];
};

#define MAX_WIEGAND_DATA	100

struct jz_wiegand_dev {
	struct cdev dev;
	unsigned int m_data;
	unsigned int m_pos;

	struct wiegand_data data[MAX_WIEGAND_DATA];

	struct list_head avail_list;
	struct list_head empty_list;
	spinlock_t avail_lock;
	spinlock_t empty_lock;

	wait_queue_head_t data_wait_queue;
	atomic_t opened;
} *jz_wiegand_devp = NULL;

static int jz_wiegand_data_available(void) {
	unsigned long flags;
	struct list_head *head;
	int avail = 0;

	spin_lock_irqsave(&jz_wiegand_devp->avail_lock, flags);
	head = &jz_wiegand_devp->avail_list;
	avail = !list_empty(head);
	spin_unlock_irqrestore(&jz_wiegand_devp->avail_lock, flags);

	return avail;
}

static struct wiegand_data *get_one_available(void) {
	unsigned long flags;
	struct wiegand_data *data = NULL;
	struct list_head *head;

	spin_lock_irqsave(&jz_wiegand_devp->avail_lock, flags);

	head = &jz_wiegand_devp->avail_list;
	if (!list_empty(head)) {
		data = list_entry(head->next, struct wiegand_data, list);
		list_del(head->next);
	}

	spin_unlock_irqrestore(&jz_wiegand_devp->avail_lock, flags);

	return data;
}

static void put_one_available(struct wiegand_data *data) {
	unsigned long flags;

	spin_lock_irqsave(&jz_wiegand_devp->avail_lock, flags);
	list_add_tail(&data->list, &jz_wiegand_devp->avail_list);
	spin_unlock_irqrestore(&jz_wiegand_devp->avail_lock, flags);

	/* wakeup any user who's waiting on this list */
	//printk("===>put one available!\n");
	wake_up_interruptible(&jz_wiegand_devp->data_wait_queue);
}

static struct wiegand_data *get_empty_data(void) {
	unsigned long flags;
	struct list_head *head = NULL;
	struct wiegand_data *curr = NULL;

	spin_lock_irqsave(&jz_wiegand_devp->empty_lock, flags);
	head = &jz_wiegand_devp->empty_list;
	if (!list_empty(head)) {
		curr = list_entry(head->next, struct wiegand_data, list);
		list_del(head->next);
	}
	spin_unlock_irqrestore(&jz_wiegand_devp->empty_lock, flags);

	if (curr)
		return curr;

	/* if no one in empty list, use avail_list's tail instead */
	spin_lock_irqsave(&jz_wiegand_devp->avail_lock, flags);
	head = &jz_wiegand_devp->avail_list;
	if (!list_empty(head)) {
		curr = list_entry(head->prev, struct wiegand_data, list);
		list_del(head->prev);
	}
	spin_unlock_irqrestore(&jz_wiegand_devp->avail_lock, flags);

	return curr;
}

static void put_to_empty_list(struct wiegand_data *data) {
	unsigned long flags;

	spin_lock_irqsave(&jz_wiegand_devp->empty_lock, flags);
	list_add(&data->list, &jz_wiegand_devp->empty_list);
	spin_unlock_irqrestore(&jz_wiegand_devp->empty_lock, flags);
}

static void wiegand_init_list(void) {
	int i;
	struct list_head *head = &jz_wiegand_devp->empty_list;

	memset(jz_wiegand_devp->data, 0, sizeof(jz_wiegand_devp->data));

	INIT_LIST_HEAD(&jz_wiegand_devp->empty_list);
	INIT_LIST_HEAD(&jz_wiegand_devp->avail_list);

	for (i = 0; i < MAX_WIEGAND_DATA; i++)
		list_add_tail(&jz_wiegand_devp->data[i].list, head);
}

/*
 * Wiegand protocal:
 * 	the Wiegand interface uses three wires,
 *	one of which is a common ground and
 *	two of which are data transmission wires usually called DATA0 and DATA1 but sometimes also labeled Data Low and Data High.
 *	When no data is being sent both DATA0 and DATA1 are at the high voltage.
 *	When a 0 is sent the Data Low wire (also called DATA0) is at a low voltage while the Data High wire stays at a high voltage.
 *	When a 1 is sent Data High is at the low voltage while Data Low stays at the high voltage
 */
#define WIEGAND_DATA0_PIN	GPD(0)
#define WIEGAND_DATA1_PIN	GPD(1)

#define WIEGAND_DATA0_IRQ	(IRQ_GPIO_0 + WIEGAND_DATA0_PIN)
#define WIEGAND_DATA1_IRQ	(IRQ_GPIO_0 + WIEGAND_DATA1_PIN)

static unsigned int wiegand_poll(struct file *filp, struct poll_table_struct *wait)
{

	poll_wait(filp, &jz_wiegand_devp->data_wait_queue, wait);
	return (jz_wiegand_data_available() ? (POLLIN | POLLRDNORM) : 0);
}

static ssize_t wiegand_read(struct file * filp,
			    char * buffer, size_t count, loff_t *ppos) {

	struct wiegand_data *data;
	long ret;

	data = get_one_available();
	if (!data) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible_timeout(jz_wiegand_devp->data_wait_queue,
						       (!list_empty(&jz_wiegand_devp->avail_list)),
						       1 * HZ);
		if (ret < 0)
			return (ssize_t)ret;

		if (ret == 0)
			return -EINVAL;

		data = get_one_available();
		if (!data)
			return -EINVAL;
	}

	if (copy_to_user(buffer, data->data, 8))
		return -EFAULT;

	put_to_empty_list(data);

	return 8;
}

static int wiegand_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int tmp;

	switch (cmd)
	{
	case WIEGAND_PULSE_WIDTH:
		break;
	case WIEGAND_PULSE_INTERVAL:
		break;
	case WIEGAND_FORMAT:
		if( get_user(tmp,(int *)arg) || (tmp < 0) || (tmp > MAX_WIEGAND_BITS))
			return -EINVAL;

		wiegand_format = tmp;
		break;
	case WIEGAND_READ:
		printk("===>please use read() instead!\n");
		return -EINVAL;
		break;
	case WIEGAND_STATUS:
		tmp = !list_empty(&jz_wiegand_devp->avail_list);
		if( put_user(tmp,(int *)arg) )
			return -EINVAL;
		break;
	default:
		printk(KERN_WARNING"JzWiegand: cmd = %d not support\n", cmd);
		return -EINVAL;

	}
	return 0;
}

//#define DEBUG_WIEGAND

static int tcu_per_ms __read_mostly;

#define TCU_LEFT_LIMIT	(tcu_per_ms - 5)
#define TCU_RIGHT_LIMIT	(tcu_per_ms + 10)

static int m_pos = 0;
static unsigned long m_data[2];
static unsigned int m_tcu_cnt[MAX_WIEGAND_BITS];
static irqreturn_t jz_wiegand_data_handler(int irq, void *dev_id) {

 restart:
	if (m_pos == 0) {
		jz_wiegand_stop_tcu_counter();
		jz_wiegand_start_tcu_counter();
		m_data[0] = 0;
		m_data[1] = 0;
	}

	m_tcu_cnt[m_pos] = jz_wiegnad_get_tcu_counter();
	if (m_pos != 0) {
		int interval = m_tcu_cnt[m_pos] - m_tcu_cnt[m_pos - 1];

		/* check fake interrupt */
		if ( interval < TCU_LEFT_LIMIT)
			return IRQ_HANDLED; /* just ack this interrupt */

		/*
		 * if intarval is greater than 20,
		 * then cheet it as beginning of another scan
		 * (and discard current data)
		 */
		if (interval > TCU_RIGHT_LIMIT) {
			m_pos = 0;
			memset(m_tcu_cnt, 0, MAX_WIEGAND_BITS * sizeof(unsigned int));
			goto restart;
		}
	}

	/* aha, data arriving ^o^ */
	if (irq == WIEGAND_DATA0_IRQ) {
#ifdef DEBUG_WIEGAND
		if (__gpio_get_pin(GPD(30)))
			__gpio_as_output0(GPD(30));
		else
			__gpio_as_output1(GPD(30));
#endif
	} else if (irq == WIEGAND_DATA1_IRQ) {
		set_bit(m_pos, m_data);
#ifdef DEBUG_WIEGAND
		if (__gpio_get_pin(GPD(31)))
			__gpio_as_output0(GPD(31));
		else
			__gpio_as_output1(GPD(31));
#endif
	} /* no else */

	m_pos++;
	if (m_pos == wiegand_format) {    /* one scan complete */
		m_pos = 0;

#ifdef DEBUG_WIEGAND
		if (m_data[0] != 0x03a54d89) {
			int i = 0;
			printk("===>scan error!, got 0x%08x\n", m_data[0]);
			for (i = 0; i < MAX_WIEGAND_BITS; i++) {
				printk("===>tcu cnt[%d] = %u\n", i, m_tcu_cnt[i]);
			}
		} else
#endif
		{
			struct wiegand_data *wdata = get_empty_data();
			wdata->data[0] = m_data[0];
			wdata->data[1] = m_data[1];
			put_one_available(wdata);
		}
		memset(m_tcu_cnt, 0, MAX_WIEGAND_BITS * sizeof(unsigned int));
	}

	return IRQ_HANDLED;
}

static int wiegand_hw_init(void) {
	int err;

	err = request_irq(WIEGAND_DATA0_IRQ, jz_wiegand_data_handler, IRQF_DISABLED, "wiegand data0", NULL);
	if (err < 0) {
		printk(KERN_ERR"request wiegand data0 irq failed!\n");
		return err;
	}

	err = request_irq(WIEGAND_DATA1_IRQ, jz_wiegand_data_handler, IRQF_DISABLED, "wiegand data1", NULL);
	if (err < 0) {
		printk(KERN_ERR"request wiegand data1 irq failed!\n");
		free_irq(WIEGAND_DATA0_IRQ, NULL);
		return err;
	}

	__gpio_as_irq_fall_edge(WIEGAND_DATA0_PIN);
	__gpio_as_irq_fall_edge(WIEGAND_DATA1_PIN);

	__gpio_as_output0(GPD(30));
	__gpio_as_output0(GPD(31));

	return 0;
}

static void wiegand_hw_fini(void) {
	free_irq(WIEGAND_DATA0_IRQ, NULL);
	free_irq(WIEGAND_DATA1_IRQ, NULL);

	/* FIXME: do we need pull? */
	__gpio_as_input(WIEGAND_DATA0_PIN);
	__gpio_as_input(WIEGAND_DATA1_PIN);
}

static int wiegand_open(struct inode * inode, struct file * filp)
{
	int ret;

	if (! atomic_dec_and_test (&jz_wiegand_devp->opened)) {
		atomic_inc(&jz_wiegand_devp->opened);
		return -EBUSY; /* already open */
	}

	filp->private_data = jz_wiegand_devp;

	ret = wiegand_hw_init();
	if (ret) {
		atomic_inc(&jz_wiegand_devp->opened);
		return ret;
	}

	/* init tcu and calculate how many TCUs per ms */
	jz_wiegand_tcu_init();
	{
		int i = 0;
		int cnt = 0;
		unsigned long flags;

		local_irq_save(flags);
		for (i = 0; i < 5; i++) {
			jz_wiegand_start_tcu_counter();
			mdelay(64);
			cnt += jz_wiegnad_get_tcu_counter();
			jz_wiegand_stop_tcu_counter();
		}
		local_irq_restore(flags);

		tcu_per_ms = cnt / 5 / 64;

		printk("===>tcu_per_ms = %d\n", tcu_per_ms);
	}

	printk("===>wiegand opened!\n");

	return 0;
}

static int wiegand_close(struct inode * inode, struct file * filp)
{
	wiegand_hw_fini();
	atomic_inc(&jz_wiegand_devp->opened); /* release the device */
	return 0;
}

static struct file_operations jz_wiegand_fops = {
	.owner = THIS_MODULE,
	.read = wiegand_read,
	.open = wiegand_open,
	.release = wiegand_close,
	.ioctl = wiegand_ioctl,
	.poll = wiegand_poll,
};

static struct class *jz_wiegand_class;

static int __init wiegand_init(void)
{
	int result;
	dev_t devno = 0;

	if (jz_wiegand_major) {
		devno = MKDEV(jz_wiegand_major, 0);
		result = register_chrdev_region(devno, 1, "wiegand");
	} else {
		result = alloc_chrdev_region(&devno, 0, 1, "wiegand");
		jz_wiegand_major = MAJOR(devno);
	}

	if (result < 0)
		return result;

	jz_wiegand_devp = kmalloc(sizeof(struct jz_wiegand_dev), GFP_KERNEL);
	if (!jz_wiegand_devp) {
		result = -ENOMEM;
		goto fail_malloc;
	}
	memset(jz_wiegand_devp, 0, sizeof(struct jz_wiegand_dev));

	spin_lock_init(&jz_wiegand_devp->avail_lock);
	spin_lock_init(&jz_wiegand_devp->empty_lock);
	atomic_set(&jz_wiegand_devp->opened, 1);
	init_waitqueue_head(&jz_wiegand_devp->data_wait_queue);
	wiegand_init_list();

	cdev_init(&jz_wiegand_devp->dev, &jz_wiegand_fops);
	jz_wiegand_devp->dev.owner = THIS_MODULE;

	result = cdev_add(&jz_wiegand_devp->dev, devno, 1);
	if (result)
		goto fail_cdev;

	jz_wiegand_class = class_create(THIS_MODULE, "wiegand");
	if (IS_ERR(jz_wiegand_class)) {
		result =  PTR_ERR(jz_wiegand_class);
		goto fail_class;
	}

	device_create(jz_wiegand_class, NULL, devno, NULL, "wiegand");
	return 0;

 fail_class:
	cdev_del(&jz_wiegand_devp->dev);
 fail_cdev:
	kfree(jz_wiegand_devp);
 fail_malloc:
	unregister_chrdev_region(devno, 1);
	return result;
}

static void __exit wiegand_exit(void)
{
	dev_t devno = MKDEV(jz_wiegand_major, 0);

	cdev_del(&jz_wiegand_devp->dev);

	device_destroy(jz_wiegand_class, devno);
	class_destroy(jz_wiegand_class);

	kfree(jz_wiegand_devp);
	unregister_chrdev_region(MKDEV(jz_wiegand_major, 0), 1);
}

module_init(wiegand_init);
module_exit(wiegand_exit);

MODULE_AUTHOR("Lutts Wolf<slcao@ingenic.cn>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("wiegand 26 bit driver"WIEGAND_VERSION);
