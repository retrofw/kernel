/*
 * linux/arch/mips/jz4760b/pm.c
 *
 * JZ4760B Power Management Routines
 *
 * Copyright (C) 2006 - 2010 Ingenic Semiconductor Inc.
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

//#define CONFIG_PM_POWERDOWN_P0 y
#define JZ_PM_SIMULATE_BATTERY y

#ifdef JZ_PM_SIMULATE_BATTERY
#define CONFIG_BATTERY_JZ
#define JZ_PM_BATTERY_SIMED
#endif

#if defined(CONFIG_RTC_DRV_JZ4760B) && defined(CONFIG_BATTERY_JZ)
//extern unsigned long jz_read_bat(void);
//extern int g_jz_battery_min_voltage;
static unsigned int usr_alarm_data = 0;
static int alarm_state = 0;
#endif

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

extern void jz_board_do_sleep(unsigned long *ptr);
extern void jz_board_do_resume(unsigned long *ptr);
#if defined(CONFIG_PM_POWERDOWN_P0)
extern void jz_cpu_sleep(void);
extern void jz_cpu_resume(void);
#endif
#if defined(CONFIG_INPUT_WM831X_ON)
extern void wm8310_power_off(void);
#endif
int jz_pm_do_hibernate(void)
{
	printk("Put CPU into hibernate mode.\n");
#if defined(CONFIG_INPUT_WM831X_ON)
	printk("The power will be off.\n");
	wm8310_power_off();
	while(1);
#else

#ifdef HP_POWER_EN
	__gpio_clear_pin(HP_POWER_EN);	//medive add
#endif
	/* Mask all interrupts */
	OUTREG32(INTC_ICMSR(0), 0xffffffff);
	OUTREG32(INTC_ICMSR(1), 0x7ff);

	/*
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 2000ms */
	rtc_write_reg(RTC_HWFCR, HWFCR_WAIT_TIME(2000));

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	rtc_write_reg(RTC_HRCR, HRCR_WAIT_TIME(60));

	/* Scratch pad register to be reserved */
	rtc_write_reg(RTC_HSPR, HSPR_RTCV);

	/* clear wakeup status register */
	rtc_write_reg(RTC_HWRSR, 0x0);

	/* set wake up valid level as low and enable rtc alarm wake up.*/
    rtc_write_reg(RTC_HWCR,0x9);

	/* Put CPU to hibernate mode */
	rtc_write_reg(RTC_HCR, HCR_PD);

	while (1) {
		printk("We should NOT come here, please check the jz4760brtc.h!!!\n");
	};
#endif

	/* We can't get here */
	return 0;
}


#if defined(CONFIG_RTC_DRV_JZ4760B) && defined(CONFIG_BATTERY_JZ)
static int alarm_remain = 0;
//#define ALARM_TIME (3 * 60)
#define ALARM_TIME (10 * 60)
static inline void jz_save_alarm(void) {
	uint32_t rtc_rtcsr = 0,rtc_rtcsar = 0;

	rtc_rtcsar = rtc_read_reg(RTC_RTCSAR); /* second alarm register */
	rtc_rtcsr = rtc_read_reg(RTC_RTCSR); /* second register */

	alarm_remain = rtc_rtcsar - rtc_rtcsr;
}

static inline void jz_restore_alarm(void) {
	if (alarm_remain > 0) {
		rtc_write_reg(RTC_RTCSAR, rtc_read_reg(RTC_RTCSR) + alarm_remain);
		rtc_set_reg(RTC_RTCCR,0x3<<2); /* alarm enable, alarm interrupt enable */
	}
}

static void jz_set_alarm(void)
{
	uint32_t rtc_rtcsr = 0,rtc_rtcsar = 0;

	rtc_rtcsar = rtc_read_reg(RTC_RTCSAR); /* second alarm register */
	rtc_rtcsr = rtc_read_reg(RTC_RTCSR); /* second register */
#if 0
	if(rtc_rtcsar <= rtc_rtcsr) {
#endif
		rtc_write_reg(RTC_RTCSAR,rtc_rtcsr + ALARM_TIME);
		rtc_set_reg(RTC_RTCCR,0x3<<2); /* alarm enable, alarm interrupt enable */
//		alarm_state = 1;	       /* alarm on */
#if 0
	} else if(rtc_rtcsar > rtc_rtcsr + ALARM_TIME) {
		printk("2\n");
		usr_alarm_data = rtc_rtcsar;
		rtc_write_reg(RTC_RTCSAR,rtc_rtcsr + ALARM_TIME);
		rtc_set_reg(RTC_RTCCR,0x3<<2);
		alarm_state = 1;
	} else {	      /* ??? I have some questions here, when the cpu is sleeping, the time freezes, doesn't it?
				 consider sleep->wakeup->sleep   --- by Lutts */
		printk("3\n");
		usr_alarm_data = 0;
		rtc_set_reg(RTC_RTCCR,0x3<<2);
		alarm_state = 0;
	}
#endif

	rtc_rtcsar = rtc_read_reg(RTC_RTCSAR); /* second alarm register */
	rtc_rtcsr = rtc_read_reg(RTC_RTCSR); /* second register */

	// printk("rtc_rtcsar=%u; rtc_rtcsr=%u; alarm_state=%d\n", rtc_rtcsar, rtc_rtcsr, alarm_state);
	printk("Suspend: on\n");
}
#undef ALARM_TIME
#endif

static int jz_pm_do_sleep(void)
{
	unsigned long nfcsr = REG_NEMC_NFCSR;
	unsigned long opcr = INREG32(CPM_OPCR);
	unsigned long icmr0 = INREG32(INTC_ICMR(0));
	unsigned long icmr1 = INREG32(INTC_ICMR(1));
	unsigned long sadc = INREG8(SADC_ADENA);
	unsigned long sleep_gpio_save[7*(GPIO_PORT_NUM-1)];
	unsigned long cpuflags;
	unsigned long addr, size, end, cnt;

#if defined(CONFIG_RTC_DRV_JZ4760B) && defined(CONFIG_BATTERY_JZ)
//	jz_save_alarm();

 __jz_pm_do_sleep_start:
#endif
 	/* set SLEEP mode */
	CMSREG32(CPM_LCR, LCR_LPM_SLEEP, LCR_LPM_MASK);

	/* Save CPU irqs */
	local_irq_save(cpuflags);

        /* Disable nand flash */
	REG_NEMC_NFCSR = ~0xff;

        /* stop sadc */
	SETREG8(SADC_ADENA,ADENA_POWER);
	while ((INREG8(SADC_ADENA) & ADENA_POWER) != ADENA_POWER) {
		dprintk("INREG8(SADC_ADENA) = 0x%x\n",INREG8(SADC_ADENA));
		udelay(100);
	}

        /* stop uhc */
	SETREG32(CPM_OPCR, OPCR_UHCPHY_DISABLE);

	/* stop otg and gps */
	CLRREG32(CPM_OPCR, OPCR_OTGPHY_ENABLE | OPCR_GPSEN);

	/*power down gps and ahb1*/
	//SETREG32(CPM_LCR, LCR_PDAHB1 | LCR_PDGPS);

	//while(!(REG_CPM_LCR && LCR_PDAHB1S)) ;
	//while(!(REG_CPM_LCR && LCR_PDGPSS)) ;

	/* Mask all interrupts except rtc*/
	OUTREG32(INTC_ICMSR(0), 0xffffffff);
	OUTREG32(INTC_ICMSR(1), 0x7fe);

#if defined(CONFIG_RTC_DRV_JZ4760B)
	OUTREG32(INTC_ICMCR(1), 0x1);

//	jz_set_alarm();
//	__intc_ack_irq(IRQ_RTC);
//	__intc_unmask_irq(IRQ_RTC);
//	rtc_clr_reg(RTC_RTCCR,RTCCR_AF);

#else
	/* mask rtc interrupts */
	OUTREG32(INTC_ICMSR(1), 0x1);
#endif

	/* Sleep on-board modules */
	jz_board_do_sleep(sleep_gpio_save);

	printk("control = 0x%08x icmr0 = 0x%08x icmr1 = 0x%08x\n",
	       INREG32(RTC_RTCCR), INREG32(INTC_ICMR(0)), INREG32(INTC_ICMR(1)));

	/* disable externel clock Oscillator in sleep mode */
	CLRREG32(CPM_OPCR, OPCR_O1SE);

	/* select 32K crystal as RTC clock in sleep mode */
	SETREG32(CPM_OPCR, OPCR_ERCS);

	/* for cpu 533M 1:2:4 */
	OUTREG32(CPM_PSWC0ST, 0);
	OUTREG32(CPM_PSWC1ST, 8);
	OUTREG32(CPM_PSWC2ST, 11);
	OUTREG32(CPM_PSWC3ST, 0);

#if defined(CONFIG_PM_POWERDOWN_P0)
	printk("Shutdown P0\n");

	cpm_set_clock(CGU_MSCCLK,JZ_EXTAL);
	cpm_start_clock(CGM_MSC0);

	/* power down the p0 */
	SETREG32(CPM_OPCR, OPCR_PD);

  	/* Clear previous reset status */
	CLRREG32(CPM_RSR, RSR_PR | RSR_WR | RSR_P0R);

   	/* Set resume return address */
	OUTREG32(CPM_CPSPPR, 0x00005a5a);
	udelay(1);
	OUTREG32(CPM_CPSPR, virt_to_phys(jz_cpu_resume));
	OUTREG32(CPM_CPSPPR, 0x0000a5a5);

	rtc_clr_reg(RTC_RTCCR,RTCCR_AF);

	/* *** go zzz *** */
	jz_cpu_sleep();
#else

	__asm__(".set\tmips32\n\t"
		"sync\n\t"
		".set\tmips32");

	/* Prefetch codes from L1 */
	addr = (unsigned long)(&&L1) & ~(32 - 1);
	size = 32 * 128; /* load 128 cachelines */
	end = addr + size;

	for (; addr < end; addr += 32) {
		__asm__ volatile (
				  ".set mips32\n\t"
				  " cache %0, 0(%1)\n\t"
				  ".set mips32\n\t"
				  :
				  : "I" (Index_Prefetch_I), "r"(addr));
	}

	/* wait for EMC stable */
	cnt = 0x3ffff;
	while(cnt--);


	/* Start of the prefetching codes */
 L1:
	*((volatile unsigned int *)0xb3020050) = 0xff00ff00;


	__asm__ volatile (".set\tmips32\n\t"
			  "wait\n\t"
			  "nop\n\t"
			  "nop\n\t"
			  "nop\n\t"
			  "nop\n\t"
			  ".set\tmips32");

	*((volatile unsigned int *)0xb3020050) = 0x0000ff00;
 L2:

	/* End of the prefetching codes */
#endif

	REG_MSC_LPM(0) = 0x1;
	/*if power down p0 ,return from sleep.S*/

	/* Restore to IDLE mode */
	CMSREG32(CPM_LCR, LCR_LPM_IDLE, LCR_LPM_MASK);

	/* Restore nand flash control register, it must be restored,
	   because it will be clear to 0 in bootrom. */
	REG_NEMC_NFCSR = nfcsr;


	/* Restore interrupts FIXME:*/
	OUTREG32(INTC_ICMR(0), icmr0);
	OUTREG32(INTC_ICMR(1), icmr1);

	/* Restore sadc */
	OUTREG8(SADC_ADENA, sadc);

	/* Resume on-board modules */
	jz_board_do_resume(sleep_gpio_save);

	/* Restore Oscillator and Power Control Register */
	OUTREG32(CPM_OPCR, opcr);

	/* Restore CPU interrupt flags */
	local_irq_restore(cpuflags);

	CLRREG32(CPM_RSR, RSR_PR | RSR_WR | RSR_P0R);

	printk("Suspend: off\n");
#if 0
#if  defined(CONFIG_RTC_DRV_JZ4760B) && defined(CONFIG_BATTERY_JZ)
	if((INREG32(RTC_RTCCR) & RTCCR_AF)) {
		rtc_clr_reg(RTC_RTCCR,RTCCR_AF | RTCCR_AE | RTCCR_AIE);
		if(!usr_alarm_data) /* restore usrs alarm state */
			rtc_write_reg(RTC_RTCSAR,usr_alarm_data);
#if 0
		if(g_jz_battery_min_voltage > jz_read_bat()) /* Just for example, add your Battery check here */
			pm_power_off();
		else
#endif
			goto __jz_pm_do_sleep_start;
	}
#endif

#if defined(CONFIG_RTC_DRV_JZ4760B) && defined(CONFIG_BATTERY_JZ)
	jz_restore_alarm();
#endif
#endif

	mdelay(50);
	while(!__gpio_get_pin(GPIO_POWER_ON) || __gpio_get_pin(UMIDO_KEY_START));

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

void jz_pm_hibernate(void)
{
	jz_pm_do_hibernate();
}

int jz_pm_sleep(void)
{
	return jz_pm_do_sleep();
}

static int jz4760b_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

/*
 * Jz CPU enter save power mode
 */
static int jz4760b_pm_enter(suspend_state_t state)
{
	jz_pm_do_sleep();
	return 0;
}

static struct platform_suspend_ops jz4760b_pm_ops = {
	.valid		= jz4760b_pm_valid,
	.enter		= jz4760b_pm_enter,
};

/*
 * Initialize power interface
 */
int __init jz_pm_init(void)
{
	printk("Power Management for JZ\n");

	suspend_set_ops(&jz4760b_pm_ops);
	return 0;
}

#ifdef JZ_PM_BATTERY_SIMED
#undef CONFIG_BATTERY_JZ
#endif

