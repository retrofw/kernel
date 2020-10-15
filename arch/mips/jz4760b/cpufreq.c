/*
 * linux/arch/mips/jz4760b/cpufreq.c
 *
 * cpufreq driver for JZ4760B 
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/cpufreq.h>

#include <asm/jzsoc.h>
#include <asm/processor.h>

#define dprintk(msg...) cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, \
						"cpufreq-jz4760b", msg)

#undef CHANGE_PLL

#define PLL_UNCHANGED 0
#define PLL_GOES_UP   1
#define PLL_GOES_DOWN 2

#define PLL_WAIT_500NS (500*(__cpm_get_cclk()/1000000000))

/* Saved the boot-time parameters */
static struct {
	/* SDRAM parameters */
	unsigned int mclk;  /* memory clock, KHz */
	unsigned int tras;  /* RAS pulse width, cycles of mclk */
	unsigned int rcd;   /* RAS to CAS Delay, cycles of mclk */
	unsigned int tpc;   /* RAS Precharge time, cycles of mclk */
	unsigned int trwl;  /* Write Precharge Time, cycles of mclk */
	unsigned int trc;   /* RAS Cycle Time, cycles of mclk */
	unsigned int rtcor; /* Refresh Time Constant */
	unsigned int sdram_initialized;

	/* LCD parameters */
	unsigned int lcdpix_clk; /* LCD Pixel clock, Hz */
	unsigned int lcd_clks_initialized;
} boot_config;

struct jz4760b_freq_percpu_info {
	struct cpufreq_frequency_table table[7];
};

static struct jz4760b_freq_percpu_info jz4760b_freq_table;

/*
 * This contains the registers value for an operating point.
 * If only part of a register needs to change then there is
 * a mask value for that register.
 * When going to a new operating point the current register
 * value is ANDed with the ~mask and ORed with the new value.
 */
struct dpm_regs {
	u32 cpccr;        /* Clock Freq Control Register */
	u32 cpccr_mask;   /* Clock Freq Control Register mask */
	u32 cppcr;        /* PLL1 Control Register */
	u32 cppcr_mask;   /* PLL1 Control Register mask */
	u32 pll_up_flag;  /* New PLL freq is higher than current or not */
};

extern jz_clocks_t jz_clocks;

static void jz_update_clocks(void)
{
	/* Next clocks must be updated if we have changed 
	 * the PLL or divisors.
	 */
	jz_clocks.cclk = __cpm_get_cclk();
	jz_clocks.hclk = __cpm_get_hclk();
	jz_clocks.mclk = __cpm_get_mclk();
	jz_clocks.pclk = __cpm_get_pclk();
	jz_clocks.pixclk = __cpm_get_pixclk();
	jz_clocks.i2sclk = __cpm_get_i2sclk();
	jz_clocks.usbclk = __cpm_get_usbclk();
	jz_clocks.mscclk = __cpm_get_mscclk(0);
}

static void
jz_init_boot_config(void)
{
	if (!boot_config.lcd_clks_initialized) {
		/* the first time to scale pll */
		boot_config.lcdpix_clk = __cpm_get_pixclk();
		boot_config.lcd_clks_initialized = 1;
	}

	if (!boot_config.sdram_initialized) {
		/* the first time to scale frequencies */
		unsigned int dmcr, rtcor;
		unsigned int tras, rcd, tpc, trwl, trc;
		
		dmcr = REG_EMC_DMCR;
		rtcor = REG_EMC_RTCOR;

		tras = (dmcr >> 13) & 0x7;
		rcd = (dmcr >> 11) & 0x3;
		tpc = (dmcr >> 8) & 0x7;
		trwl = (dmcr >> 5) & 0x3;
		trc = (dmcr >> 2) & 0x7;

		boot_config.mclk = __cpm_get_mclk() / 1000;
		boot_config.tras = tras + 4;
		boot_config.rcd = rcd + 1;
		boot_config.tpc = tpc + 1;
		boot_config.trwl = trwl + 1;
		boot_config.trc = trc * 2 + 1;
		boot_config.rtcor = rtcor;

		boot_config.sdram_initialized = 1;
	}
}

static void jz_update_dram_rtcor(unsigned int new_mclk)
{
	unsigned int rtcor;
	
	new_mclk /= 1000;
	rtcor = boot_config.rtcor * new_mclk / boot_config.mclk;
	rtcor--;

	if (rtcor < 1) rtcor = 1;
	if (rtcor > 255) rtcor = 255;

	REG_EMC_RTCOR = rtcor;
	REG_EMC_RTCNT = rtcor;
}

static void jz_update_dram_dmcr(unsigned int new_mclk)
{
	unsigned int dmcr;
	unsigned int tras, rcd, tpc, trwl, trc;
	unsigned int valid_time, new_time; /* ns */

	new_mclk /= 1000;
	tras = boot_config.tras * new_mclk / boot_config.mclk;
	rcd = boot_config.rcd * new_mclk / boot_config.mclk;
	tpc = boot_config.tpc * new_mclk / boot_config.mclk;
	trwl = boot_config.trwl * new_mclk / boot_config.mclk;
	trc = boot_config.trc * new_mclk / boot_config.mclk;

	/* Validation checking */
	valid_time = (boot_config.tras * 1000000) / boot_config.mclk;
	new_time = (tras * 1000000) / new_mclk;
	if (new_time < valid_time) tras += 1;

	valid_time = (boot_config.rcd * 1000000) / boot_config.mclk;
	new_time = (rcd * 1000000) / new_mclk;
	if (new_time < valid_time) rcd += 1;

	valid_time = (boot_config.tpc * 1000000) / boot_config.mclk;
	new_time = (tpc * 1000000) / new_mclk;
	if (new_time < valid_time) tpc += 1;

	valid_time = (boot_config.trwl * 1000000) / boot_config.mclk;
	new_time = (trwl * 1000000) / new_mclk;
	if (new_time < valid_time) trwl += 1;

	valid_time = (boot_config.trc * 1000000) / boot_config.mclk;
	new_time = (trc * 1000000) / new_mclk;
	if (new_time < valid_time) trc += 2;

	tras = (tras < 4) ? 4: tras;
	tras = (tras > 11) ? 11: tras;
	tras -= 4;

	rcd = (rcd < 1) ? 1: rcd;
	rcd = (rcd > 4) ? 4: rcd;
	rcd -= 1;

	tpc = (tpc < 1) ? 1: tpc;
	tpc = (tpc > 8) ? 8: tpc;
	tpc -= 1;

	trwl = (trwl < 1) ? 1: trwl;
	trwl = (trwl > 4) ? 4: trwl;
	trwl -= 1;

	trc = (trc < 1) ? 1: trc;
	trc = (trc > 15) ? 15: trc;
	trc /= 2;	

	dmcr = REG_EMC_DMCR;
	
	dmcr &= ~(EMC_DMCR_TRAS_MASK | EMC_DMCR_RCD_MASK | EMC_DMCR_TPC_MASK | EMC_DMCR_TRWL_MASK | EMC_DMCR_TRC_MASK);
	dmcr |= ((tras << EMC_DMCR_TRAS_BIT) | (rcd << EMC_DMCR_RCD_BIT) | (tpc << EMC_DMCR_TPC_BIT) | (trwl << EMC_DMCR_TRWL_BIT) | (trc << EMC_DMCR_TRC_BIT));

	REG_EMC_DMCR = dmcr;
}

static void jz_update_dram_prev(unsigned int cur_mclk, unsigned int new_mclk)
{	
	/* No risk, no fun: run with interrupts on! */
	if (new_mclk > cur_mclk) {
		/* We're going FASTER, so first update TRAS, RCD, TPC, TRWL
		 * and TRC of DMCR before changing the frequency.
		 */
		jz_update_dram_dmcr(new_mclk);
	} else {
		/* We're going SLOWER: first update RTCOR value
		 * before changing the frequency.
		 */
		jz_update_dram_rtcor(new_mclk);
	}
}

static void jz_update_dram_post(unsigned int cur_mclk, unsigned int new_mclk)
{	
	/* No risk, no fun: run with interrupts on! */
	if (new_mclk > cur_mclk) {
		/* We're going FASTER, so update RTCOR
		 * after changing the frequency 
		 */
		jz_update_dram_rtcor(new_mclk);
	} else {
		/* We're going SLOWER: so update TRAS, RCD, TPC, TRWL
		 * and TRC of DMCR after changing the frequency.
		 */
		jz_update_dram_dmcr(new_mclk);
	}
}

static void jz_scale_divisors(struct dpm_regs *regs)
{
	unsigned int cpccr;
	unsigned int cur_mclk, new_mclk;
	int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned int tmp = 0, wait = PLL_WAIT_500NS; 

	cpccr = REG_CPM_CPCCR;
	cpccr &= ~((unsigned long)regs->cpccr_mask);
	cpccr |= regs->cpccr;
	cpccr |= CPM_CPCCR_CE;       /* update immediately */

	cur_mclk = __cpm_get_mclk();
	new_mclk = __cpm_get_pllout() / div[(cpccr & CPM_CPCCR_MDIV_MASK) >> CPM_CPCCR_MDIV_BIT];

	/* Update some DRAM parameters before changing frequency */
	jz_update_dram_prev(cur_mclk, new_mclk);

	/* update register to change the clocks.
	 * align this code to a cache line.
	 */
	__asm__ __volatile__(
		".set noreorder\n\t"
		".align 5\n"
		"sw %1,0(%0)\n\t"
		"li %3,0\n\t"
		"1:\n\t"
		"bne %3,%2,1b\n\t"
		"addi %3, 1\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		".set reorder\n\t"
		:
		: "r" (CPM_CPCCR), "r" (cpccr), "r" (wait), "r" (tmp));

	/* Update some other DRAM parameters after changing frequency */
	jz_update_dram_post(cur_mclk, new_mclk);
}

#ifdef CHANGE_PLL
/* Maintain the LCD clock and pixel clock */
static void jz_scale_lcd_divisors(struct dpm_regs *regs)
{	
	unsigned int new_pll, new_lcd_div, new_lcdpix_div;
	unsigned int cpccr;
	unsigned int tmp = 0, wait = PLL_WAIT_500NS; 

	if (!boot_config.lcd_clks_initialized) return;

	new_pll = __cpm_get_pllout();
	new_lcd_div = new_pll / boot_config.lcd_clk;
	new_lcdpix_div = new_pll / boot_config.lcdpix_clk;

	if (new_lcd_div < 1)
		new_lcd_div = 1;
	if (new_lcd_div > 16)
		new_lcd_div = 16;

	if (new_lcdpix_div < 1)
		new_lcdpix_div = 1;
	if (new_lcdpix_div > 512)
		new_lcdpix_div = 512;

//	REG_CPM_CPCCR2 = new_lcdpix_div - 1;

	cpccr = REG_CPM_CPCCR;
	cpccr &= ~CPM_CPCCR_LDIV_MASK;
	cpccr |= ((new_lcd_div - 1) << CPM_CPCCR_LDIV_BIT);
	cpccr |= CPM_CPCCR_CE;       /* update immediately */

	/* update register to change the clocks.
	 * align this code to a cache line.
	 */
	__asm__ __volatile__(
		".set noreorder\n\t"
		".align 5\n"
		"sw %1,0(%0)\n\t"
		"li %3,0\n\t"
		"1:\n\t"
		"bne %3,%2,1b\n\t"
		"addi %3, 1\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		".set reorder\n\t"
		:
		: "r" (CPM_CPCCR), "r" (cpccr), "r" (wait), "r" (tmp));
}

static void jz_scale_pll(struct dpm_regs *regs)
{
	unsigned int cppcr;
	unsigned int cur_mclk, new_mclk, new_pll;
	int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	int od[] = {1, 2, 2, 4};

	cppcr = REG_CPM_CPPCR;
	cppcr &= ~(regs->cppcr_mask | CPM_CPPCR_PLLS | CPM_CPPCR_PLLEN | CPM_CPPCR_PLLST_MASK);
	regs->cppcr &= ~CPM_CPPCR_PLLEN;
	cppcr |= (regs->cppcr | 0xff);

	/* Update some DRAM parameters before changing frequency */
	new_pll = JZ_EXTAL * ((cppcr>>23)+2) / ((((cppcr>>18)&0x1f)+2) * od[(cppcr>>16)&0x03]);
	cur_mclk = __cpm_get_mclk();
	new_mclk = new_pll / div[(REG_CPM_CPCCR>>16) & 0xf];

	/*
	 * Update some SDRAM parameters
	 */
	jz_update_dram_prev(cur_mclk, new_mclk);

	/* 
	 * Update PLL, align code to cache line.
	 */
	cppcr |= CPM_CPPCR_PLLEN;
	__asm__ __volatile__(
		".set noreorder\n\t"
		".align 5\n"
		"sw %1,0(%0)\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		".set reorder\n\t"
		:
		: "r" (CPM_CPPCR), "r" (cppcr));

	/* Update some other DRAM parameters after changing frequency */
	jz_update_dram_post(cur_mclk, new_mclk);
}
#endif

static void jz4760b_transition(struct dpm_regs *regs)
{
	/*
	 * Get and save some boot-time conditions.
	 */
	jz_init_boot_config();

#ifdef CHANGE_PLL
	/* 
	 * Disable LCD before scaling pll.
	 * LCD and LCD pixel clocks should not be changed even if the PLL 
	 * output frequency has been changed.
	 */
	REG_LCD_CTRL &= ~LCD_CTRL_ENA;

	/*
	 * Stop module clocks before scaling PLL
	 */
	__cpm_stop_eth();
	__cpm_stop_aic(1);
	__cpm_stop_aic(2);
#endif

	/* ... add more as necessary */

	if (regs->pll_up_flag == PLL_GOES_UP) {
		/* the pll frequency is going up, so change dividors first */
		jz_scale_divisors(regs);
#ifdef CHANGE_PLL
		jz_scale_pll(regs);
#endif
	}
	else if (regs->pll_up_flag == PLL_GOES_DOWN) {
		/* the pll frequency is going down, so change pll first */
#ifdef CHANGE_PLL
		jz_scale_pll(regs);
#endif
		jz_scale_divisors(regs);
	}
	else {
		/* the pll frequency is unchanged, so change divisors only */
		jz_scale_divisors(regs);
	}

#ifdef CHANGE_PLL
	/*
	 * Restart module clocks before scaling PLL
	 */
	__cpm_start_eth();
	__cpm_start_aic(1);
	__cpm_start_aic(2);

	/* ... add more as necessary */

	/* Scale the LCD divisors after scaling pll */
	if (regs->pll_up_flag != PLL_UNCHANGED) {
		jz_scale_lcd_divisors(regs);
	}

	/* Enable LCD controller */
	REG_LCD_CTRL &= ~LCD_CTRL_DIS;
	REG_LCD_CTRL |= LCD_CTRL_ENA;
#endif

	/* Update system clocks */
	jz_update_clocks();
}

extern unsigned int idle_times;
static unsigned int jz4760b_freq_get(unsigned int cpu)
{
	return  (__cpm_get_cclk() / 1000);
}

static unsigned int index_to_divisor(unsigned int index, struct dpm_regs *regs)
{
	int n2FR[33] = {
		0, 0, 1, 2, 3, 0, 4, 0, 5, 0, 0, 0, 6, 0, 0, 0,
		7, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0,
		9
	};
	int div[4] = {1, 2, 2, 2}; /* divisors of I:S:P:M */
	unsigned int div_of_cclk, new_freq, i;

	regs->pll_up_flag = PLL_UNCHANGED;
	regs->cpccr_mask = CPM_CPCCR_CDIV_MASK | CPM_CPCCR_HDIV_MASK | CPM_CPCCR_PDIV_MASK | CPM_CPCCR_MDIV_MASK;

	new_freq = jz4760b_freq_table.table[index].frequency;

	do {
		div_of_cclk = __cpm_get_pllout() / (1000 * new_freq);
	} while (div_of_cclk==0);

	if(div_of_cclk == 1 || div_of_cclk == 2 || div_of_cclk == 4) {
		for(i = 1; i<4; i++) {
			div[i] = 3;
		}
	} else {
		for(i = 1; i<4; i++) {
			div[i] = 2;
		}
	}

	for(i = 0; i<4; i++) {
		div[i] *= div_of_cclk;
	}

	dprintk("divisors of I:S:P:M = %d:%d:%d:%d\n", div[0], div[1], div[2], div[3]);

	regs->cpccr = 
		(n2FR[div[0]] << CPM_CPCCR_CDIV_BIT) | 
		(n2FR[div[1]] << CPM_CPCCR_HDIV_BIT) | 
		(n2FR[div[2]] << CPM_CPCCR_PDIV_BIT) |
		(n2FR[div[3]] << CPM_CPCCR_MDIV_BIT);

	return  div_of_cclk;
}

static void jz4760b_set_cpu_divider_index(unsigned int cpu, unsigned int index)
{
	unsigned long divisor, old_divisor;
	struct cpufreq_freqs freqs;
	struct dpm_regs regs;

	old_divisor = __cpm_get_pllout() /  __cpm_get_cclk();
	divisor = index_to_divisor(index, &regs);

	freqs.old = __cpm_get_cclk() / 1000;
	freqs.new =  __cpm_get_pllout() / (1000 * divisor);
	freqs.cpu = cpu;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	if (old_divisor != divisor)
		jz4760b_transition(&regs);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
}

static int jz4760b_freq_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	unsigned int new_index = 0;

	if (cpufreq_frequency_table_target(policy,
					   &jz4760b_freq_table.table[0],
					   target_freq, relation, &new_index))
		return -EINVAL;

	jz4760b_set_cpu_divider_index(policy->cpu, new_index);

	dprintk("new frequency is %d KHz (REG_CPM_CPCCR:0x%x)\n", __cpm_get_cclk() / 1000, REG_CPM_CPCCR);

	return 0;
}

static int jz4760b_freq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy,
					      &jz4760b_freq_table.table[0]);
}

static int __init jz4760b_cpufreq_driver_init(struct cpufreq_policy *policy)
{

	struct cpufreq_frequency_table *table =	&jz4760b_freq_table.table[0];
	unsigned int MAX_FREQ;

	dprintk(KERN_INFO "Jz4760b cpufreq driver\n");

	if (policy->cpu != 0)
		return -EINVAL;

	policy->cur = MAX_FREQ = __cpm_get_cclk() / 1000; /* in kHz. Current and max frequency is determined by u-boot */
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;

	policy->cpuinfo.min_freq = MAX_FREQ/8;
	policy->cpuinfo.max_freq = MAX_FREQ;
	policy->cpuinfo.transition_latency = 100000; /* in 10^(-9) s = nanoseconds */

	table[0].index = 0;
	table[0].frequency = MAX_FREQ/8;
	table[1].index = 1;
	table[1].frequency = MAX_FREQ/6;
	table[2].index = 2;
	table[2].frequency = MAX_FREQ/4;
	table[3].index = 3;
	table[3].frequency = MAX_FREQ/3;
	table[4].index = 4;
	table[4].frequency = MAX_FREQ/2;
	table[5].index = 5;
	table[5].frequency = MAX_FREQ;
	table[6].index = 6;
	table[6].frequency = CPUFREQ_TABLE_END;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(table, policy->cpu); /* for showing /sys/devices/system/cpu/cpuX/cpufreq/stats/ */
#endif

	return  cpufreq_frequency_table_cpuinfo(policy, table);
}

static struct cpufreq_driver cpufreq_jz4760b_driver = {
//	.flags		= CPUFREQ_STICKY,
	.init		= jz4760b_cpufreq_driver_init,
	.verify		= jz4760b_freq_verify,
	.target		= jz4760b_freq_target,
	.get		= jz4760b_freq_get,
	.name		= "jz4760b",
};

static int __init jz4760b_cpufreq_init(void)
{
	return cpufreq_register_driver(&cpufreq_jz4760b_driver);
}

static void __exit jz4760b_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&cpufreq_jz4760b_driver);
}

module_init(jz4760b_cpufreq_init);
module_exit(jz4760b_cpufreq_exit);

MODULE_AUTHOR("Regen <lhhuang@ingenic.cn>");
MODULE_DESCRIPTION("cpufreq driver for Jz4760b");
MODULE_LICENSE("GPL");
