#ifndef __JZ_TS_H__
#define __JZ_TS_H__

/* 
 * IOCTL commands
 */
#define IOCTL_SET_MSG  0
#define IOCTL_SET_NUM  1


/*
 * TS Event type
 */
struct ts_event {
	u16 status;
        u16 x;
        u16 y;
        u16 pressure;
        u16 pad;
};

/* TS event status */
#define PENUP    0x00
#define PENDOWN  0x01

#define EVENT_BUFSIZE 64 // must be power of two

struct jz_ts_t {
	int pendown_irq;       // IRQ of pendown interrupt
	int pen_is_down;       // 1 = pen is down, 0 = pen is up
	int irq_enabled;
	struct ts_event event_buf[EVENT_BUFSIZE];// The event queue
	int nextIn, nextOut;
	int event_count;
	struct fasync_struct *fasync;     // asynch notification
	struct timer_list acq_timer;      // Timer for triggering acquisitions
	wait_queue_head_t wait;           // read wait queue
	spinlock_t lock;
        int minx, miny, maxx, maxy;
        int filter, prints;
	int sleeping;
	int first_read;
};

extern void ts_enable_irq(void);
extern void ts_disable_irq(void);
extern int ts_request_irq(u32 *irq,irqreturn_t (*handler)(int, void *), const char *devname, void *dev_id);
extern void ts_free_irq(struct jz_ts_t *ts);
extern int PenIsDown(void);
extern int AcquireEvent(struct jz_ts_t *ts, struct ts_event *event);
extern void ts_irq_callback(void);
extern void ts_data_ready(void);

#endif /* __JZ_TS_H__ */
