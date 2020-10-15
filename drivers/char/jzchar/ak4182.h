#ifndef __AK4182_H__
#define __AK4182_H__

/* Device data structure */

struct ak4182 {
	spinlock_t		lock;
	struct pm_dev		*pmdev;
	struct semaphore	adc_sem;
	u16			adc_cr;
	u16			irq_fal_enbl;
	u16			irq_ris_enbl;
	int			irq_enabled;
};

#endif /* __AK4182_H__ */
