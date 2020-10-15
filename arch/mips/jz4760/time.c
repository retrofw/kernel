/*
 * linux/arch/mips/jz4760/time.c
 *
 * Setting up the clock on the JZ4760 boards.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/clockchips.h>

#include <asm/time.h>
#include <asm/jzsoc.h>

/* This is for machines which generate the exact clock. */

#define JZ_TIMER_TCU_CH  5
#define JZ_TIMER_IRQ  IRQ_TCU1

#define JZ_TIMER_CLOCK (JZ_EXTAL>>4) /* Jz timer clock frequency */

static struct clocksource clocksource_jz; /* Jz clock source */
static struct clock_event_device jz_clockevent_device; /* Jz clock event */

void (*jz_timer_callback)(void);

static irqreturn_t jz_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd = dev_id;

	__tcu_clear_full_match_flag(JZ_TIMER_TCU_CH);

	if (jz_timer_callback)
		jz_timer_callback();

	cd->event_handler(cd);

	return IRQ_HANDLED;
}

static struct irqaction jz_irqaction = {
	.handler	= jz_timer_interrupt,
	.flags		= IRQF_DISABLED | IRQF_PERCPU | IRQF_TIMER,
	.name		= "jz-timerirq",
};

static unsigned int current_cycle_high = 0;

union clycle_type
{
	cycle_t cycle64;
	unsigned int cycle32[2];
};

cycle_t jz_get_cycles(struct clocksource *unused)
{
	/* convert jiffes to jz timer cycles */
	unsigned int ostcount;
	unsigned long cpuflags;
	unsigned int current_cycle;
	unsigned int flag;
	union clycle_type old_cycle;

	local_irq_save(cpuflags);
	current_cycle = current_cycle_high;
	ostcount = REG_OST_OSTCNT;
	flag = (REG_TCU_TFR & TFCR_OSTFLAG) ? 1: 0;
	if(flag)
		ostcount = REG_OST_OSTCNT;
	local_irq_restore(cpuflags);

	old_cycle.cycle32[0] = ostcount;
	old_cycle.cycle32[1] = current_cycle + flag;

	return (old_cycle.cycle64);
}

static struct clocksource clocksource_jz = {
	.name 		= "jz_clocksource",
	.rating		= 300,
	.read		= jz_get_cycles,
	.mask		= 0xFFFFFFFF,
	.shift 		= 10,
	.flags		= CLOCK_SOURCE_WATCHDOG,
};

static irqreturn_t jzclock_handler(int irq, void *dev_id)
{
	REG_TCU_TFCR = TFCR_OSTFLAG; /* ACK timer */
	current_cycle_high++;

	return IRQ_HANDLED;
}

static struct irqaction jz_clockaction = {
	.handler	= jzclock_handler,
	.flags		= IRQF_DISABLED  | IRQF_TIMER,
	.name		= "jz-clockcycle",
};
static int __init jz_clocksource_init(void)
{
	unsigned int latch;

	/* Init timer */
	latch = (JZ_TIMER_CLOCK + (HZ>>1)) / HZ;

	clocksource_jz.mult = clocksource_hz2mult(JZ_TIMER_CLOCK, clocksource_jz.shift);
	clocksource_register(&clocksource_jz);

	//---------------------init sys clock -----------------
	REG_OST_OSTCSR = OSTCSR_PRESCALE16 | OSTCSR_EXT_EN;
	REG_OST_OSTDR = 0xffffffff;
	REG_OST_OSTCNT = 0;

	jz_clockaction.dev_id = &clocksource_jz;

	setup_irq(IRQ_TCU0, &jz_clockaction);
	REG_TCU_TMCR = TMCR_OSTMASK; /* unmask match irq */
	REG_TCU_TSCR = TSCR_OST;  /* enable timer clock */
	REG_TCU_TESR = TESR_OST;  /* start counting up */

	//---------------------endif init sys clock -----------------

	return 0;
}

static int jz_set_next_event(unsigned long evt,
				  struct clock_event_device *unused)
{
	return 0;
}

static void jz_set_mode(enum clock_event_mode mode,
			struct clock_event_device *evt)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
                break;
        case CLOCK_EVT_MODE_ONESHOT:
        case CLOCK_EVT_MODE_UNUSED:
        case CLOCK_EVT_MODE_SHUTDOWN:
                break;
        case CLOCK_EVT_MODE_RESUME:
                break;
        }
}

static struct clock_event_device jz_clockevent_device = {
	.name		= "jz-clockenvent",
	.features	= CLOCK_EVT_FEAT_PERIODIC,
//	.features	= CLOCK_EVT_FEAT_ONESHOT, /* Jz4740 not support dynamic clock now */

	/* .mult, .shift, .max_delta_ns and .min_delta_ns left uninitialized */
	.mult		= 1,
	.rating		= 300,
	.irq		= JZ_TIMER_IRQ,
	.set_mode	= jz_set_mode,
	.set_next_event	= jz_set_next_event,
};

static void __init jz_clockevent_init(void)
{
	struct clock_event_device *cd = &jz_clockevent_device;
	unsigned int cpu = smp_processor_id();

	cd->cpumask = cpumask_of(cpu);
	clockevents_register_device(cd);
}

static void __init jz_timer_setup(void)
{
  	unsigned int latch;

	jz_clocksource_init();	/* init jz clock source */
	jz_clockevent_init();	/* init jz clock event */

	//---------------------init sys tick -----------------
	/* Init timer */
	__tcu_stop_counter(JZ_TIMER_TCU_CH);
//	__cpm_start_tcu();
	latch = (JZ_TIMER_CLOCK + (HZ>>1)) / HZ;

	REG_TCU_TMSR = ((1 << JZ_TIMER_TCU_CH) | (1 << (JZ_TIMER_TCU_CH + 16)));

	REG_TCU_TCSR(JZ_TIMER_TCU_CH) = TCSR_PRESCALE16 | TCSR_EXT_EN;
	REG_TCU_TDFR(JZ_TIMER_TCU_CH) = latch - 1;
	REG_TCU_TDHR(JZ_TIMER_TCU_CH) = latch + 1;
	REG_TCU_TCNT(JZ_TIMER_TCU_CH) = 0;
	/*
	 * Make irqs happen for the system timer
	 */
	jz_irqaction.dev_id = &jz_clockevent_device;
	setup_irq(JZ_TIMER_IRQ, &jz_irqaction);
	__tcu_clear_full_match_flag(JZ_TIMER_TCU_CH);
	__tcu_unmask_full_match_irq(JZ_TIMER_TCU_CH);
	__tcu_start_counter(JZ_TIMER_TCU_CH);}


void __init plat_time_init(void)
{
	jz_timer_setup();
}
