/*
 * jz_ts.c
 *
 * Touch screen driver for the Ingenic JZ47XX.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>

#include "jz_ts.h"

MODULE_AUTHOR("Peter Wei <jlwei@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic Touch Screen Driver");
MODULE_LICENSE("GPL");

#define TS_NAME "jz-ts"
#define TS_MINOR 16   /* MAJOR: 10, MINOR: 16 */
#define PFX TS_NAME

//#define JZ_TS_DEBUG

#ifdef JZ_TS_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG PFX ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) printk(KERN_ERR PFX ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format "\n" , ## arg)

static struct jz_ts_t jz_ts;

unsigned int (*codec_read_battery)(void) = NULL;

// hold the spinlock before calling.
static void event_add(struct jz_ts_t *ts, struct ts_event *event)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->lock, flags);

	// add this event to the event queue
	ts->event_buf[ts->nextIn] = *event;
	ts->nextIn = (ts->nextIn + 1) & (EVENT_BUFSIZE - 1);
	if (ts->event_count < EVENT_BUFSIZE) {
		ts->event_count++;
	} else {
		// throw out the oldest event
		ts->nextOut = (ts->nextOut + 1) & (EVENT_BUFSIZE - 1);
	}

	spin_unlock_irqrestore(&ts->lock, flags);

	// async notify
	if (ts->fasync)
		kill_fasync(&ts->fasync, SIGIO, POLL_IN);
	// wake up any read call
	if (waitqueue_active(&ts->wait))
		wake_up_interruptible(&ts->wait);
}

static int event_pull(struct jz_ts_t *ts, struct ts_event *event)
{
	unsigned long flags;
	int ret;
	
	spin_lock_irqsave(&ts->lock, flags);
	ret = ts->event_count;
	if (ts->event_count) {
		*event = ts->event_buf[ts->nextOut];
		ts->nextOut = (ts->nextOut + 1) & (EVENT_BUFSIZE - 1);
		ts->event_count--;
	}
	spin_unlock_irqrestore(&ts->lock, flags);

	return ret;
}

static int pen_is_down = 0;

static irqreturn_t pendown_interrupt(int irq, void * dev_id)
{
	struct jz_ts_t* ts = &jz_ts;
	struct ts_event event;

	dbg("pen down");
#if defined(CONFIG_SOC_JZ4740)
	if (ts->sleeping) {
		ts->sleeping = 0;
		ts_data_ready();
		return IRQ_HANDLED;
		}
#endif
	spin_lock(&ts->lock);

	if (ts->irq_enabled) {
		ts->irq_enabled = 0;
	}
	else
		ts->irq_enabled = 1;

	
	if (pen_is_down)
		pen_is_down = 0;
	else
		pen_is_down = 1;
	
	// callback routine to clear irq status
	ts_irq_callback();

	if ( (pen_is_down == 0)){
		del_timer(&ts->acq_timer);
		spin_unlock(&ts->lock);
		event.x = event.y = event.pressure = 0;
		event.status = PENUP;
		ts->first_read = 0;
		event_add(ts, &event);
		return IRQ_HANDLED;
	}

	if ( (pen_is_down == 1))
	{
		ts->acq_timer.expires = jiffies + HZ / 100;
		del_timer(&ts->acq_timer);
		ts->first_read = 1;
		add_timer(&ts->acq_timer);
		spin_unlock(&ts->lock);
	}
	return IRQ_HANDLED;
}


/*
 * Raw X,Y,pressure acquisition timer function. It gets scheduled
 * only while pen is down. Its duration between calls is the polling
 * rate.
 */
static void
jz_acq_timer(unsigned long data)
{
	struct jz_ts_t *ts = (struct jz_ts_t *)data;
	struct ts_event event;
	int pen_was_down = ts->pen_is_down;

	spin_lock(&ts->lock);

	if (PenIsDown()) {

		ts->pen_is_down = 1;

		if (AcquireEvent(ts, &event)) // check event is valid or not?
			event_add(ts, &event);

		// schedule next acquire
		ts->acq_timer.expires = jiffies + HZ / 100;
		del_timer(&ts->acq_timer);
		add_timer(&ts->acq_timer);
	} else {

		if (!ts->irq_enabled) {
			ts->irq_enabled = 1;
		}
		ts->pen_is_down = 0;
		if (pen_was_down) {
			event.x = event.y = event.pressure = 0;
			event.status = PENUP;
			event_add(ts, &event);
		}
	}

	spin_unlock(&ts->lock);
}

/* +++++++++++++ Read battery voltage routine ++++++++++++++*/

unsigned int jz_read_battery(void)
{
	unsigned int v = 0;
	struct jz_ts_t *ts = &jz_ts;

	spin_lock(&ts->lock);

	if (codec_read_battery)
		v = codec_read_battery();

	spin_unlock(&ts->lock);

	return v;
}

/* +++++++++++++ File operations ++++++++++++++*/

static int
jz_fasync(int fd, struct file *filp, int mode)
{
	struct jz_ts_t *ts = (struct jz_ts_t *)filp->private_data;
	return fasync_helper(fd, filp, mode, &ts->fasync);
}


static unsigned int
jz_poll(struct file * filp, poll_table * wait)
{
	struct jz_ts_t* ts = (struct jz_ts_t*)filp->private_data;
	poll_wait(filp, &ts->wait, wait);
	if (ts->event_count)
		return POLLIN | POLLRDNORM;
	return 0;
}

static ssize_t
jz_read(struct file * filp, char * buffer, size_t count, loff_t * ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct jz_ts_t* ts = (struct jz_ts_t*)filp->private_data;
	char *ptr = buffer;
	struct ts_event event;
	int err = 0;

	dbg("jz_read");

	add_wait_queue(&ts->wait, &wait);
	while (count >= sizeof(struct ts_event)) {
		err = -ERESTARTSYS;
		if (signal_pending(current))
			break;


		if (event_pull(ts, &event)) {
			err = copy_to_user(ptr, &event,
					   sizeof(struct ts_event));
			if (err)
				break;
			ptr += sizeof(struct ts_event);
			count -= sizeof(struct ts_event);
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
			err = -EAGAIN;
			if (filp->f_flags & O_NONBLOCK)
				break; 
			schedule();
		}
	}
	
	current->state = TASK_RUNNING;
	remove_wait_queue(&ts->wait, &wait);
	
	return ptr == buffer ? err : ptr - buffer;
}


static int
jz_open(struct inode * inode, struct file * filp)
{
	struct jz_ts_t *ts;
	int retval;

	dbg("open ts device");
	filp->private_data = ts = &jz_ts;

	spin_lock(&ts->lock);

	ts->pen_is_down = 0; // start with pen up
	ts->sleeping = 0;
	// flush event queue
	ts->nextIn = ts->nextOut = ts->event_count = 0;
	
	// Init acquisition timer function
	init_timer(&ts->acq_timer);
	ts->acq_timer.function = jz_acq_timer;
	ts->acq_timer.data = (unsigned long)ts;

	ts->irq_enabled = 1;

	spin_unlock(&ts->lock);

	/* Since ts interrupt can happen immediately after request_irq,
	 * we wait until we've completed init of all relevent driver
	 * state variables. Now we grab the PenDown IRQ
	 */
	retval = ts_request_irq(&ts->pendown_irq, pendown_interrupt, TS_NAME, ts);
	if (retval) {
		err("unable to get PenDown IRQ %d", ts->pendown_irq);
		return retval;
	}

	try_module_get(THIS_MODULE);
	return 0;
}

static int
jz_release(struct inode * inode, struct file * filp)
{
	struct jz_ts_t* ts = (struct jz_ts_t*)filp->private_data;

	ts_free_irq(ts);
	jz_fasync(-1, filp, 0);
	del_timer_sync(&ts->acq_timer);

 	module_put(THIS_MODULE);
	return 0;
}

static int jz_ioctl(struct inode *inode, struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
	struct txy {
		int minx;
		int miny;
		int maxx;
		int maxy; 
	};

	struct txy  ch;

	/* 
	 * Switch according to the ioctl called 
	 */
	switch (ioctl_num)
	{
	case IOCTL_SET_MSG:
		jz_ts.filter=1;
          	break;
        case IOCTL_SET_NUM:
		if (copy_from_user((void *)&ch, (void *)ioctl_param, sizeof(ch)))
			return -EFAULT;	
                jz_ts.minx = ch.minx;
		jz_ts.miny = ch.miny;
		jz_ts.maxx = ch.maxx;
		jz_ts.maxy = ch.maxy; 
        	break;
	}

	return 0;
}

static struct file_operations ts_fops = {
	owner:          THIS_MODULE,
	read:           jz_read,
	poll:           jz_poll,
	fasync:         jz_fasync,
	ioctl:          jz_ioctl,
	open:		jz_open,
	release:	jz_release,
};

/* +++++++++++++ End File operations ++++++++++++++*/

static int __init minx_setup(char *str)
{
	int i;

	if (get_option(&str,&i)) jz_ts.minx = i;
	jz_ts.filter=i;
	return 1;
}

__setup("ts_minx=", minx_setup);

static int __init miny_setup(char *str)
{
	int i;
	if (get_option(&str,&i)) jz_ts.miny = i;	
	return 1;
}

__setup("ts_miny=", miny_setup);

static int __init maxx_setup(char *str)
{
	int i;
	if (get_option(&str,&i)) jz_ts.maxx = i;	
	return 1;
}

__setup("ts_maxx=", maxx_setup);

static int __init maxy_setup(char *str)
{
	int i;
	if (get_option(&str,&i)) jz_ts.maxy = i;
	return 1;
}

__setup("ts_maxy=", maxy_setup);

static int __init printraw_setup(char *str)
{
	if (str)
	       jz_ts.prints = 1;

	return 0;
}

__setup("ts_debug", printraw_setup);


static struct miscdevice jz_ts_dev = {
	minor:  TS_MINOR,
	name:   TS_NAME,
	fops:   &ts_fops,
};

static int __init jzts_init_module(void)
{
	struct jz_ts_t *ts = &jz_ts;
	int ret;
	
	if ((ret = misc_register(&jz_ts_dev)) < 0) {
		err("can't register misc device");
		return ret;
	}

//	memset(ts, 0, sizeof(struct jz_ts_t));
	init_waitqueue_head(&ts->wait);
	spin_lock_init(&ts->lock);

	printk(JZ_SOC_NAME ": Generic touch screen driver registered.\n");

	return 0;
}

static void jzts_cleanup_module(void)
{
	misc_deregister(&jz_ts_dev);
}

module_init(jzts_init_module);
module_exit(jzts_cleanup_module);
