/*
 * linux/arch/mips/jz4810/common/pm.c
 *
 * JZ4810 Power Management Routines
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
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

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>

#include <asm/cacheops.h>
#include <asm/jzsoc.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

#define GPIO_PORT_NUM   6

extern void jz_cpu_sleep(void);
extern void jz_cpu_resume(void);

/*
 * __gpio_as_sleep set all pins to pull-disable, and set all pins as input
 * except sdram and the pins which can be used as CS1_N to CS4_N for chip select.
 */
#define __gpio_as_sleep()	              \
do {	                                      \
	REG_GPIO_PXFUNC(1) = ~0x03ff7fff;     \
	REG_GPIO_PXSELC(1) = ~0x03ff7fff;     \
	REG_GPIO_PXDIRC(1) = ~0x03ff7fff;     \
	REG_GPIO_PXPES(1)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(2) = ~0x01e00000;     \
	REG_GPIO_PXSELC(2) = ~0x01e00000;     \
	REG_GPIO_PXDIRC(2) = ~0x01e00000;     \
	REG_GPIO_PXPES(2)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(3) =  0xffffffff;     \
	REG_GPIO_PXSELC(3) =  0xffffffff;     \
	REG_GPIO_PXDIRC(3) =  0xffffffff;     \
	REG_GPIO_PXPES(3)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(4) =  0xffffffff;     \
	REG_GPIO_PXSELC(4) =  0xffffffff;     \
	REG_GPIO_PXDIRC(4) =  0xffffffff;     \
	REG_GPIO_PXPES(4)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(5) =  0xffffffff;     \
	REG_GPIO_PXSELC(5) =  0xffffffff;     \
	REG_GPIO_PXDIRC(5) =  0xffffffff;     \
	REG_GPIO_PXPES(5)  =  0xffffffff;     \
} while (0)


static int jz_pm_do_hibernate(void)
{
	printk("Put CPU into hibernate mode.\n");

	/* Mask all interrupts */
	REG_INTC_IMSR(0) = 0xffffffff;
	REG_INTC_IMSR(1) = 0xffffffff;

	/*
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */
#if 0
	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWFCR = (100 << RTC_HWFCR_BIT);

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HRCR = (60 << RTC_HRCR_BIT); /* 60 ms */

	/* Scratch pad register to be reserved */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HSPR = 0x12345678;

	/* clear wakeup status register */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWRSR = 0x0;

	/* Put CPU to power down mode */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HCR = RTC_HCR_PD;

	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	while(1);
#endif

	/* We can't get here */
	return 0;
}

/* NOTES:
 * 1: Pins that are floated (NC) should be set as input and pull-enable.
 * 2: Pins that are pull-up or pull-down by outside should be set as input
 *    and pull-disable.
 * 3: Pins that are connected to a chip except sdram and nand flash
 *    should be set as input and pull-disable, too.
 */
static void jz_board_do_sleep(unsigned long *ptr)
{
#if 0
	unsigned char i;

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("run dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n",        \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}

        /* Save GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		*ptr++ = REG_GPIO_PXFUN(i);
		*ptr++ = REG_GPIO_PXSEL(i);
		*ptr++ = REG_GPIO_PXDIR(i);
		*ptr++ = REG_GPIO_PXPE(i);
		*ptr++ = REG_GPIO_PXIM(i);
		*ptr++ = REG_GPIO_PXDAT(i);
		*ptr++ = REG_GPIO_PXTRG(i);
	}

        /*
         * Set all pins to pull-disable, and set all pins as input except
         * sdram and the pins which can be used as CS1_N to CS4_N for chip select.
         */
        __gpio_as_sleep();

        /*
	 * Set proper status for GPC21 to GPC24 which can be used as CS1_N to CS4_N.
	 * Keep the pins' function used for chip select(CS) here according to your
         * system to avoid chip select crashing with sdram when resuming from sleep mode.
         */

#if defined(CONFIG_JZ4810_APUS)
        /* GPB25/CS1_N is used as chip select for nand flash, shouldn't be change. */

        /* GPB26/CS2_N is connected to nand flash, needn't be changed. */

        /* GPB28/CS3_N is used as cs8900's chip select, shouldn't be changed. */

	/* GPB27/CS4_N is used as NOR's chip select, shouldn't be changed. */
#endif

 	/*
         * Enable pull for NC pins here according to your system
	 */

#if defined(CONFIG_JZ4810_APUS)
#endif

	/*
         * If you must set some GPIOs as output to high level or low level,
         * you can set them here, using:
         * __gpio_as_output(n);
         * __gpio_set_pin(n); or  __gpio_clear_pin(n);
	 */

#if defined(CONFIG_JZ4810_APUS)
	/* GPC7 which is used as AMPEN_N should be set to high to disable audio amplifier */
	__gpio_as_output(32*2+7);
	__gpio_set_pin(32*2+7);
#endif

#ifdef DEBUG
        /* Keep uart function for printing debug message */
	__gpio_as_uart0();
	__gpio_as_uart1();
	__gpio_as_uart2();
	__gpio_as_uart3();

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("sleep dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n",      \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
#endif
#endif
}

static void jz_board_do_resume(unsigned long *ptr)
{
#if 0
	unsigned char i;

	/* Restore GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		 REG_GPIO_PXFUNS(i) = *ptr;
		 REG_GPIO_PXFUNC(i) = ~(*ptr++);

		 REG_GPIO_PXSELS(i) = *ptr;
		 REG_GPIO_PXSELC(i) = ~(*ptr++);

		 REG_GPIO_PXDIRS(i) = *ptr;
		 REG_GPIO_PXDIRC(i) = ~(*ptr++);

		 REG_GPIO_PXPES(i) = *ptr;
		 REG_GPIO_PXPEC(i) = ~(*ptr++);

		 REG_GPIO_PXIMS(i)=*ptr;
		 REG_GPIO_PXIMC(i)=~(*ptr++);

		 REG_GPIO_PXDATS(i)=*ptr;
		 REG_GPIO_PXDATC(i)=~(*ptr++);

		 REG_GPIO_PXTRGS(i)=*ptr;
		 REG_GPIO_PXTRGC(i)=~(*ptr++);
	}

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		dprintk("resume dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n",     \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
#endif
}

static int jz_pm_do_sleep(void)
{
#if 1
	unsigned long delta;
	unsigned long imr0 = REG_INTC_IMR(0);
	unsigned long imr1 = REG_INTC_IMR(1);
	unsigned long opcr = REG_CPM_OPCR;
#if 0
	unsigned long nfcsr = REG_EMC_NFCSR;

	unsigned long sadc = REG_SADC_ENA;
	unsigned long pmembs0 = REG_EMC_PMEMBS0;
	unsigned long sleep_gpio_save[7*(GPIO_PORT_NUM-1)];
#endif
	unsigned long cpuflags;

	printk("Put CPU into sleep mode.\n");

	CMSREG32(CPM_LCR, CPM_LCR_LPM_SLEEP, CPM_LCR_LPM_MASK);

	/* Preserve current time */
	delta = xtime.tv_sec - REG_RTC_RSR;

	/* Save CPU irqs */
	local_irq_save(cpuflags);

        /* Disable nand flash */
	//REG_EMC_NFCSR = ~0xff;

	/*pull up enable pin of DQS */
	//REG_EMC_PMEMBS0 |= (0xff << 8);

        /* stop sadc */
	//REG_SADC_ENA &= ~0x7;
	//while((REG_SADC_ENA & 0x7) != 0);
 	//udelay(100);

        /*stop udc and usb*/
	//__cpm_suspend_uhc_phy();
	//__cpm_suspend_otg_phy();

	/*stop gps*/
	//__cpm_suspend_gps();

	/* Mask all interrupts */
	REG_INTC_IMSR(0) = 0xffffffff;
	REG_INTC_IMSR(1) = 0xffffffff;

	/* Sleep on-board modules */
	//jz_board_do_sleep(sleep_gpio_save);

#if 0
	/* Just allow following interrupts to wakeup the system.
	 * Note: modify this according to your system.
	 */

	/* enable RTC alarm */
	__intc_unmask_irq(IRQ_RTC);

        /* make system wake up after n seconds by RTC alarm */
	unsigned int v, n;
	n = 5;
	while (!__rtc_write_ready());
	__rtc_enable_alarm();
	while (!__rtc_write_ready());
	__rtc_enable_alarm_irq();
 	while (!__rtc_write_ready());
 	v = __rtc_get_second();
 	while (!__rtc_write_ready());
 	__rtc_set_alarm_second(v+n);


#endif
	/* WAKEUP key */
	/* PD17: boot_sel[0] */
	__gpio_as_irq_fall_edge(32 * 3 + 17);
	__gpio_ack_irq(32 * 3 + 17);
	__gpio_unmask_irq(32 * 3 + 17);
	__intc_unmask_irq(IRQ_GPIO0 - ((32 * 3 + 17)/32));  /* unmask IRQ_GPIOn depends on GPIO_WAKEUP */

	REG_GPIO_PXFLGC(3) = 0xffffffff;
	printk("===>int = 0x%08x\n", REG_GPIO_PXINT(3));
        printk("===>mask = 0x%08x\n", REG_GPIO_PXMASK(3));
        printk("===>pat1 = 0x%08x\n", REG_GPIO_PXPAT1(3));
        printk("===>pat0 = 0x%08x\n", REG_GPIO_PXPAT0(3));
        printk("===>flag = 0x%08x\n", REG_GPIO_PXFLG(3));

	printk("gpio_level = %d IPR0 = 0x%08x IPR1 = 0x%08x\n",
		__gpio_get_pin(32 * 3 + 17), REG_INTC_IPR(0), REG_INTC_IPR(1));

	printk("===>enter sleeping ......\n");
#if 0
	printk("Shutdown P0 ......\n");
	/*power down the p0*/
	REG_CPM_OPCR |= CPM_OPCR_PD;

	/* disable externel clock Oscillator in sleep mode */
	__cpm_disable_osc_in_sleep();
	/* select 32K crystal as RTC clock in sleep mode */
	__cpm_select_rtcclk_rtc();

       	/* Clear previous reset status */
	REG_CPM_RSR &= ~(CPM_RSR_PR | CPM_RSR_WR | CPM_RSR_P0R);

       	/* Set resume return address */
	REG_CPM_CPPSR = virt_to_phys(jz_cpu_resume);

	/* *** go zzz *** */
	jz_cpu_sleep();

#else

	__asm__(".set\tmips3\n\t"
		"sync\n\t"
		"wait\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		".set\tmips0");
#endif

	/*if power down p0 ,return from sleep.S*/
	printk("gpio_level = %d IPR0 = 0x%08x IPR1 = 0x%08x\n",
		__gpio_get_pin(32 * 3 + 17), REG_INTC_IPR(0), REG_INTC_IPR(1));
	__gpio_ack_irq(32 * 3 + 17);

	/* Restore to IDLE mode */
	CMSREG32(CPM_LCR, CPM_LCR_LPM_IDLE, CPM_LCR_LPM_MASK);

        /* Restore nand flash control register */
	//REG_EMC_NFCSR = nfcsr;

	/*Restore pmembs0*/
	//REG_EMC_PMEMBS0 = pmembs0;

	/* Restore interrupts */
	REG_INTC_IMSR(0) = imr0;
	REG_INTC_IMSR(1) = imr1;

	REG_INTC_IMCR(0) = ~imr0;
	REG_INTC_IMCR(1) = ~imr1;

	/* Restore sadc */
	//REG_SADC_ENA = sadc;

	/* Resume on-board modules */
	//jz_board_do_resume(sleep_gpio_save);

	/* Restore Oscillator and Power Control Register */
	REG_CPM_OPCR = opcr;

	/* Restore CPU interrupt flags */
	local_irq_restore(cpuflags);

	/* Restore current time */
	xtime.tv_sec = REG_RTC_RSR + delta;

	printk("Resume CPU from sleep mode.\n");
#endif
	return 0;
}

#define K0BASE  KSEG0
void jz_flush_cache_all(void)
{
	unsigned long addr;

	/* Clear CP0 TagLo */
	asm volatile ("mtc0 $0, $28\n\t"::);

	for (addr = K0BASE; addr < (K0BASE + 0x4000); addr += 32) {
		asm volatile (
			".set mips3\n\t"
			" cache %0, 0(%1)\n\t"
			".set mips2\n\t"
			:
			: "I" (Index_Writeback_Inv_D), "r"(addr));

		asm volatile (
			".set mips3\n\t"
			" cache %0, 0(%1)\n\t"
			".set mips2\n\t"
			:
			: "I" (Index_Store_Tag_I), "r"(addr));
	}

	asm volatile ("sync\n\t"::);

	/* invalidate BTB */
	asm volatile (
		".set mips32\n\t"
		" mfc0 %0, $16, 7\n\t"
		" nop\n\t"
		" ori $0, 2\n\t"
		" mtc0 %0, $16, 7\n\t"
		" nop\n\t"
		".set mips2\n\t"
		:
		: "r"(addr));
}

#if 0
#ifndef CONFIG_JZ_POWEROFF
static irqreturn_t pm_irq_handler (int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
#endif

/* Put CPU to SLEEP mode */
int jz_pm_sleep(void)
{
	int retval;

#ifndef CONFIG_JZ_POWEROFF
	if ((retval = request_irq (IRQ_GPIO_0 + GPIO_WAKEUP, pm_irq_handler, IRQF_DISABLED,
				   "PM", NULL))) {
		printk ("PM could not get IRQ for GPIO_WAKEUP\n");
		return retval;
	}
#endif

	retval = jz_pm_do_sleep();

#ifndef CONFIG_JZ_POWEROFF
	free_irq (IRQ_GPIO_0 + GPIO_WAKEUP, NULL);
#endif

	return retval;
}
#endif

/* Put CPU to HIBERNATE mode
 *----------------------------------------------------------------------------
 * Power Management sleep sysctl interface
 *
 * Write "mem" to /sys/power/state invokes this function
 * which initiates a poweroff.
 */
void jz_pm_hibernate(void)
{
	jz_pm_do_hibernate();
}

/* Put CPU to SLEEP mode
 *----------------------------------------------------------------------------
 * Power Management sleep sysctl interface
 *
 * Write "standby" to /sys/power/state invokes this function
 * which initiates a sleep.
 */

int jz_pm_sleep(void)
{
	return jz_pm_do_sleep();
}

/*
 * valid states, only support standby(sleep) and mem(hibernate)
 */
static int jz4810_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

/*
 * Jz CPU enter save power mode
 */
static int jz4810_pm_enter(suspend_state_t state)
{
	return jz_pm_do_sleep();
}

static struct platform_suspend_ops jz4810_pm_ops = {
	.valid		= jz4810_pm_valid,
	.enter		= jz4810_pm_enter,
};

/*
 * Initialize power interface
 */
int __init jz_pm_init(void)
{
	printk("Power Management for JZ\n");

	suspend_set_ops(&jz4810_pm_ops);
	return 0;
}

