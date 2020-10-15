/*
 * jz4760btcu.h
 * JZ4760B TCU register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: fyliao@ingenic.cn
 */

#ifndef __JZ4760BTCU_H__
#define __JZ4760BTCU_H__


/*
 * Timer and counter unit module(TCU) address definition
 */
#define	TCU_BASE		0xb0002000

/* TCU group offset */
#define TCU_GOS			0x10

/* TCU total channel number */
#define TCU_CHANNEL_NUM		8


/*
 * TCU registers offset definition
 */
#define TCU_TER_OFFSET		(0x10)  /* r, 16, 0x0000     */
#define TCU_TESR_OFFSET		(0x14)  /* w, 16, 0x????     */
#define TCU_TECR_OFFSET		(0x18)  /* w, 16, 0x????     */
#define TCU_TSR_OFFSET		(0x1c)  /* r, 32, 0x00000000 */
#define TCU_TFR_OFFSET		(0x20)  /* r, 32, 0x003F003F */
#define TCU_TFSR_OFFSET		(0x24)  /* w, 32, 0x???????? */
#define TCU_TFCR_OFFSET		(0x28)  /* w, 32, 0x???????? */
#define TCU_TSSR_OFFSET		(0x2c)  /* w, 32, 0x00000000 */
#define TCU_TMR_OFFSET		(0x30)  /* r, 32, 0x00000000 */
#define TCU_TMSR_OFFSET		(0x34)  /* w, 32, 0x???????? */
#define TCU_TMCR_OFFSET		(0x38)  /* w, 32, 0x???????? */
#define TCU_TSCR_OFFSET		(0x3c)  /* w, 32, 0x0000     */

#define TCU_TDFR_OFFSET		(0x40)  /* rw,16, 0x????     */
#define TCU_TDHR_OFFSET		(0x44)  /* rw,16, 0x????     */
#define TCU_TCNT_OFFSET		(0x48)  /* rw,16, 0x????     */
#define TCU_TCSR_OFFSET		(0x4c)  /* rw,16, 0x0000     */

#define TCU_TSTR_OFFSET		(0xf0)  /* r, 32, 0x00000000 */
#define TCU_TSTSR_OFFSET	(0xf4)  /* w, 32, 0x???????? */
#define TCU_TSTCR_OFFSET	(0xf8)  /* w, 32, 0x???????? */


/*
 * TCU registers address definition
 */
#define TCU_TER		(TCU_BASE + TCU_TER_OFFSET)
#define TCU_TESR	(TCU_BASE + TCU_TESR_OFFSET)
#define TCU_TECR	(TCU_BASE + TCU_TECR_OFFSET)
#define TCU_TSR		(TCU_BASE + TCU_TSR_OFFSET)
#define TCU_TFR		(TCU_BASE + TCU_TFR_OFFSET)
#define TCU_TFSR	(TCU_BASE + TCU_TFSR_OFFSET)
#define TCU_TFCR	(TCU_BASE + TCU_TFCR_OFFSET)
#define TCU_TSSR	(TCU_BASE + TCU_TSSR_OFFSET)
#define TCU_TMR		(TCU_BASE + TCU_TMR_OFFSET)
#define TCU_TMSR	(TCU_BASE + TCU_TMSR_OFFSET)
#define TCU_TMCR	(TCU_BASE + TCU_TMCR_OFFSET)
#define TCU_TSCR	(TCU_BASE + TCU_TSCR_OFFSET)
#define TCU_TSTR	(TCU_BASE + TCU_TSTR_OFFSET)
#define TCU_TSTSR	(TCU_BASE + TCU_TSTSR_OFFSET)
#define TCU_TSTCR	(TCU_BASE + TCU_TSTCR_OFFSET)

/* n is the TCU channel index (0 - 7) */
#define TCU_TDFR(n)	(TCU_BASE + (n) * TCU_GOS + TCU_TDFR_OFFSET)
#define TCU_TDHR(n)	(TCU_BASE + (n) * TCU_GOS + TCU_TDHR_OFFSET)
#define TCU_TCNT(n)	(TCU_BASE + (n) * TCU_GOS + TCU_TCNT_OFFSET)
#define TCU_TCSR(n)	(TCU_BASE + (n) * TCU_GOS + TCU_TCSR_OFFSET)


/*
 * TCU registers bit field common define
 */

/* When n is NOT less than TCU_CHANNEL_NUM, change to TCU_CHANNEL_NUM - 1 */
#define __TIMER(n)	(1 << ((n) < TCU_CHANNEL_NUM ? (n) : (TCU_CHANNEL_NUM - 1)))

/* Timer counter enable register(TER) */
#define TER_OSTEN	BIT15
#define TER_TCEN(n)	__TIMER(n)

/* Timer counter enable set register(TESR) */
#define TESR_OST	BIT15
#define TESR_TIMER(n)	__TIMER(n)

/* Timer counter enable clear register(TECR) */
#define TECR_OST	BIT15
#define TECR_TIMER(n)	__TIMER(n)

/* Timer stop register(TSR) */
#define TSR_WDT_STOP		BIT16
#define TSR_OST_STOP		BIT15
#define TSR_TIMER_STOP(n)	__TIMER(n)

/* Timer stop set register(TSSR) */
#define TSSR_WDT		BIT16
#define TSSR_OST		BIT15
#define TSSR_TIMER(n)		__TIMER(n)

/* Timer stop clear register(TSCR) */
#define TSCR_WDT		BIT16
#define TSCR_OST		BIT15
#define TSSR_TIMER(n)		__TIMER(n)

/* Timer flag register(TFR) */
#define TFR_HFLAG(n)	(__TIMER(n) << 16)
#define TFR_OSTFLAG	BIT15
#define TFR_FFLAG(n)	__TIMER(n)

/* Timer flag set register(TFSR) */
#define TFSR_HFLAG(n)	(__TIMER(n) << 16)
#define TFSR_OSTFLAG	BIT15
#define TFSR_FFLAG(n)	__TIMER(n)

/* Timer flag clear register(TFCR) */
#define TFCR_HFLAG(n)	(__TIMER(n) << 16)
#define TFCR_OSTFLAG	BIT15
#define TFCR_FFLAG(n)	(__TIMER(n))

/* Timer mast register(TMR) */
#define TMR_HMASK(n)	(__TIMER(n) << 16)
#define TMR_OSTMASK	BIT15
#define TMR_FMASK(n)	(__TIMER(n))

/* Timer mask set register(TMSR) */
#define TMSR_HMASK(n)	(__TIMER(n) << 16)
#define TMSR_OSTMASK	BIT15
#define TMSR_FMASK(n)	(__TIMER(n))

/* Timer mask clear register(TMCR) */
#define TMCR_HMASK(n)	(__TIMER(n) << 16)
#define TMCR_OSTMASK	BIT15
#define TMCR_FMASK(n)	(__TIMER(n))

/* Timer control register(TCSR) */
#define TCSR_CLRZ		BIT10
#define TCSR_SD_ABRUPT		BIT9
#define TCSR_INITL_HIGH		BIT8
#define TCSR_PWM_EN		BIT7
#define TCSR_PWM_IN_EN		BIT6
#define TCSR_EXT_EN		BIT2
#define TCSR_RTC_EN		BIT1
#define TCSR_PCK_EN		BIT0

#define TCSR_PRESCALE_LSB	3
#define TCSR_PRESCALE_MASK	BITS_H2L(5, TCSR_PRESCALE_LSB)
#define TCSR_PRESCALE1		(0x0 << TCSR_PRESCALE_LSB)
#define TCSR_PRESCALE4		(0x1 << TCSR_PRESCALE_LSB)
#define TCSR_PRESCALE16		(0x2 << TCSR_PRESCALE_LSB)
#define TCSR_PRESCALE64		(0x3 << TCSR_PRESCALE_LSB)
#define TCSR_PRESCALE256	(0x4 << TCSR_PRESCALE_LSB)
#define TCSR_PRESCALE1024	(0x5 << TCSR_PRESCALE_LSB)

/* Timer data full register(TDFR) */
#define TDFR_TDFR_LSB		0
#define TDFR_TDFR_MASK		BITS_H2L(15, TDFR_TDFR_LSB)

/* Timer data half register(TDHR) */
#define TDHR_TDHR_LSB		0
#define TDHR_TDHR_MASK		BITS_H2L(15, TDHR_TDHR_LSB)

/* Timer counter register(TCNT) */
#define TCNT_TCNT_LSB		0
#define TCNT_TCNT_MASK		BITS_H2L(15, TCNT_TCNT_LSB)

/* Timer status register(TSTR) */
#define TSTR_REAL2	BIT18
#define TSTR_REAL1	BIT17
#define TSTR_BUSY2	BIT2
#define TSTR_BUSY1	BIT1

/* Timer status set register(TSTSR) */
#define TSTSR_REALS2	BIT18
#define TSTSR_REALS1	BIT17
#define TSTSR_BUSYS2	BIT2
#define TSTSR_BUSYS1	BIT1

/* Timer status clear register(TSTCR) */
#define TSTCR_REALC2	BIT18
#define TSTCR_REALC1	BIT17
#define TSTCR_BUSYC2	BIT2
#define TSTCR_BUSYC1	BIT1


#ifndef __MIPS_ASSEMBLER

#define REG_TCU_TER	REG16(TCU_TER)
#define REG_TCU_TESR	REG16(TCU_TESR)
#define REG_TCU_TECR	REG16(TCU_TECR)
#define REG_TCU_TSR	REG32(TCU_TSR)
#define REG_TCU_TFR	REG32(TCU_TFR)
#define REG_TCU_TFSR	REG32(TCU_TFSR)
#define REG_TCU_TFCR	REG32(TCU_TFCR)
#define REG_TCU_TSSR	REG32(TCU_TSSR)
#define REG_TCU_TMR	REG32(TCU_TMR)
#define REG_TCU_TMSR	REG32(TCU_TMSR)
#define REG_TCU_TMCR	REG32(TCU_TMCR)
#define REG_TCU_TSCR	REG32(TCU_TSCR)
#define REG_TCU_TSTR	REG32(TCU_TSTR)
#define REG_TCU_TSTSR	REG32(TCU_TSTSR)
#define REG_TCU_TSTCR	REG32(TCU_TSTCR)

#define REG_TCU_TDFR(n)	REG16(TCU_TDFR(n))
#define REG_TCU_TDHR(n)	REG16(TCU_TDHR(n))
#define REG_TCU_TCNT(n)	REG16(TCU_TCNT(n))
#define REG_TCU_TCSR(n)	REG16(TCU_TCSR(n))


// where 'n' is the TCU channel
#define __tcu_select_extalclk(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~(TCSR_EXT_EN | TCSR_RTC_EN | TCSR_PCK_EN)) | TCSR_EXT_EN)
#define __tcu_select_rtcclk(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~(TCSR_EXT_EN | TCSR_RTC_EN | TCSR_PCK_EN)) | TCSR_RTC_EN)
#define __tcu_select_pclk(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~(TCSR_EXT_EN | TCSR_RTC_EN | TCSR_PCK_EN)) | TCSR_PCK_EN)
#define __tcu_disable_pclk(n) \
	REG_TCU_TCSR(n) = (REG_TCU_TCSR((n)) & ~TCSR_PCK_EN);
#define __tcu_select_clk_div1(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCSR_PRESCALE_MASK) | TCSR_PRESCALE1)
#define __tcu_select_clk_div4(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCSR_PRESCALE_MASK) | TCSR_PRESCALE4)
#define __tcu_select_clk_div16(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCSR_PRESCALE_MASK) | TCSR_PRESCALE16)
#define __tcu_select_clk_div64(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCSR_PRESCALE_MASK) | TCSR_PRESCALE64)
#define __tcu_select_clk_div256(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCSR_PRESCALE_MASK) | TCSR_PRESCALE256)
#define __tcu_select_clk_div1024(n) \
	(REG_TCU_TCSR((n)) = (REG_TCU_TCSR((n)) & ~TCSR_PRESCALE_MASK) | TCSR_PRESCALE1024)

#define __tcu_enable_pwm_output(n)	(REG_TCU_TCSR((n)) |= TCSR_PWM_EN)
#define __tcu_disable_pwm_output(n)	(REG_TCU_TCSR((n)) &= ~TCSR_PWM_EN)

#define __tcu_init_pwm_output_high(n)	(REG_TCU_TCSR((n)) |= TCSR_INITL_HIGH)
#define __tcu_init_pwm_output_low(n)	(REG_TCU_TCSR((n)) &= ~TCSR_INITL_HIGH)

#define __tcu_set_pwm_output_shutdown_graceful(n)	(REG_TCU_TCSR((n)) &= ~TCSR_SD_ABRUPT)
#define __tcu_set_pwm_output_shutdown_abrupt(n)		(REG_TCU_TCSR((n)) |= TCSR_SD_ABRUPT)

#define __tcu_clear_counter_to_zero(n)	(REG_TCU_TCSR((n)) |= TCSR_CLRZ)

#define __tcu_ost_enabled()		(REG_TCU_TER & TER_OSTEN)
#define __tcu_enable_ost()		(REG_TCU_TESR = TESR_OST)
#define __tcu_disable_ost()		(REG_TCU_TECR = TECR_OST)

#define __tcu_counter_enabled(n)	(REG_TCU_TER & (1 << (n)))
#define __tcu_start_counter(n)		(REG_TCU_TESR |= (1 << (n)))
#define __tcu_stop_counter(n)		(REG_TCU_TECR |= (1 << (n)))

#define __tcu_half_match_flag(n)	(REG_TCU_TFR & (1 << ((n) + 16)))
#define __tcu_full_match_flag(n)	(REG_TCU_TFR & (1 << (n)))
#define __tcu_set_half_match_flag(n)	(REG_TCU_TFSR = (1 << ((n) + 16)))
#define __tcu_set_full_match_flag(n)	(REG_TCU_TFSR = (1 << (n)))
#define __tcu_clear_half_match_flag(n)	(REG_TCU_TFCR = (1 << ((n) + 16)))
#define __tcu_clear_full_match_flag(n)	(REG_TCU_TFCR = (1 << (n)))
#define __tcu_mask_half_match_irq(n)	(REG_TCU_TMSR = (1 << ((n) + 16)))
#define __tcu_mask_full_match_irq(n)	(REG_TCU_TMSR = (1 << (n)))
#define __tcu_unmask_half_match_irq(n)	(REG_TCU_TMCR = (1 << ((n) + 16)))
#define __tcu_unmask_full_match_irq(n)	(REG_TCU_TMCR = (1 << (n)))

#define __tcu_ost_match_flag()		(REG_TCU_TFR & TFR_OSTFLAG)
#define __tcu_set_ost_match_flag()	(REG_TCU_TFSR = TFSR_OSTFLAG)
#define __tcu_clear_ost_match_flag()	(REG_TCU_TFCR = TFCR_OSTFLAG)
#define __tcu_ost_match_irq_masked()	(REG_TCU_TMR & TMR_OSTMASK)
#define __tcu_mask_ost_match_irq()	(REG_TCU_TMSR = TMSR_OSTMASK)
#define __tcu_unmask_ost_match_irq()	(REG_TCU_TMCR = TMCR_OSTMASK)

#define __tcu_wdt_clock_stopped()	(REG_TCU_TSR & TSR_WDT_STOP)
#define __tcu_ost_clock_stopped()	(REG_TCU_TSR & TSR_OST_STOP)
#define __tcu_timer_clock_stopped(n)	(REG_TCU_TSR & (1 << (n)))

#define __tcu_start_wdt_clock()		(REG_TCU_TSCR = TSCR_WDT)
#define __tcu_start_ost_clock()		(REG_TCU_TSCR = TSCR_OST)
#define __tcu_start_timer_clock(n)	(REG_TCU_TSCR = (1 << (n)))

#define __tcu_stop_wdt_clock()		(REG_TCU_TSSR = TSSR_WDT)
#define __tcu_stop_ost_clock()		(REG_TCU_TSSR = TSSR_OST)
#define __tcu_stop_timer_clock(n)	(REG_TCU_TSSR = (1 << (n)))

#define __tcu_get_count(n)		(REG_TCU_TCNT((n)))
#define __tcu_set_count(n,v)		(REG_TCU_TCNT((n)) = (v))
#define __tcu_set_full_data(n,v)	(REG_TCU_TDFR((n)) = (v))
#define __tcu_set_half_data(n,v)	(REG_TCU_TDHR((n)) = (v))

/* TCU2, counter 1, 2*/
#define __tcu_read_real_value(n)	(REG_TCU_TSTR & (1 << ((n) + 16)))
#define __tcu_read_false_value(n)	(REG_TCU_TSTR & (1 << ((n) + 16)))
#define __tcu_counter_busy(n)		(REG_TCU_TSTR & (1 << (n)))
#define __tcu_counter_ready(n)		(REG_TCU_TSTR & (1 << (n)))

#define __tcu_set_read_real_value(n)	(REG_TCU_TSTSR = (1 << ((n) + 16)))
#define __tcu_set_read_false_value(n)	(REG_TCU_TSTCR = (1 << ((n) + 16)))
#define __tcu_set_counter_busy(n)	(REG_TCU_TSTSR = (1 << (n)))
#define __tcu_set_counter_ready(n)	(REG_TCU_TSTCR = (1 << (n)))

#endif /* __MIPS_ASSEMBLER */


#endif /* __JZ4760BTCU_H__ */
