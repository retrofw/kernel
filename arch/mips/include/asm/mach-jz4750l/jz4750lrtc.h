/*
 * linux/include/asm-mips/mach-jz4750l/jz4750lrtc.h
 *
 * JZ4750L RTC register definition
 *
 * Copyright (C) 2008-2014 Ingenic Semiconductor Co., Ltd.
 */
#ifndef __JZ4750LRTC_H__
#define __JZ4750LRTC_H__

/* Define the module base addresses */
#define	RTC_BASE	0xB0003000

/*************************************************************************
 * RTC
 *************************************************************************/
#define RTC_RCR		(RTC_BASE + 0x00) /* RTC Control Register */
#define RTC_RSR		(RTC_BASE + 0x04) /* RTC Second Register */
#define RTC_RSAR	(RTC_BASE + 0x08) /* RTC Second Alarm Register */
#define RTC_RGR		(RTC_BASE + 0x0c) /* RTC Regulator Register */

#define RTC_HCR		(RTC_BASE + 0x20) /* Hibernate Control Register */
#define RTC_HWFCR	(RTC_BASE + 0x24) /* Hibernate Wakeup Filter Counter Reg */
#define RTC_HRCR	(RTC_BASE + 0x28) /* Hibernate Reset Counter Register */
#define RTC_HWCR	(RTC_BASE + 0x2c) /* Hibernate Wakeup Control Register */
#define RTC_HWRSR	(RTC_BASE + 0x30) /* Hibernate Wakeup Status Register */
#define RTC_HSPR	(RTC_BASE + 0x34) /* Hibernate Scratch Pattern Register */

#define REG_RTC_RCR	REG32(RTC_RCR)
#define REG_RTC_RSR	REG32(RTC_RSR)
#define REG_RTC_RSAR	REG32(RTC_RSAR)
#define REG_RTC_RGR	REG32(RTC_RGR)
#define REG_RTC_HCR	REG32(RTC_HCR)
#define REG_RTC_HWFCR	REG32(RTC_HWFCR)
#define REG_RTC_HRCR	REG32(RTC_HRCR)
#define REG_RTC_HWCR	REG32(RTC_HWCR)
#define REG_RTC_HWRSR	REG32(RTC_HWRSR)
#define REG_RTC_HSPR	REG32(RTC_HSPR)

/* RTC Control Register */
#define RTC_RCR_WRDY_BIT 	7
#define RTC_RCR_WRDY		(1 << 7)  /* Write Ready Flag */
#define RTC_RCR_1HZ_BIT		6
#define RTC_RCR_1HZ		(1 << RTC_RCR_1HZ_BIT)  /* 1Hz Flag */
#define RTC_RCR_1HZIE		(1 << 5)  /* 1Hz Interrupt Enable */
#define RTC_RCR_AF_BIT		4
#define RTC_RCR_AF		(1 << RTC_RCR_AF_BIT)  /* Alarm Flag */
#define RTC_RCR_AIE		(1 << 3)  /* Alarm Interrupt Enable */
#define RTC_RCR_AE		(1 << 2)  /* Alarm Enable */
#define RTC_RCR_RTCE		(1 << 0)  /* RTC Enable */

/* RTC Regulator Register */
#define RTC_RGR_LOCK		(1 << 31) /* Lock Bit */
#define RTC_RGR_ADJC_BIT	16
#define RTC_RGR_ADJC_MASK	(0x3ff << RTC_RGR_ADJC_BIT)
#define RTC_RGR_NC1HZ_BIT	0
#define RTC_RGR_NC1HZ_MASK	(0xffff << RTC_RGR_NC1HZ_BIT)

/* Hibernate Control Register */
#define RTC_HCR_PD		(1 << 0)  /* Power Down */

/* Hibernate Wakeup Filter Counter Register */
#define RTC_HWFCR_BIT		5
#define RTC_HWFCR_MASK		(0x7ff << RTC_HWFCR_BIT)

/* Hibernate Reset Counter Register */
#define RTC_HRCR_BIT		5
#define RTC_HRCR_MASK		(0x7f << RTC_HRCR_BIT)

/* Hibernate Wakeup Control Register */
#define RTC_HWCR_EALM		(1 << 0)  /* RTC alarm wakeup enable */

/* Hibernate Wakeup Status Register */
#define RTC_HWRSR_HR		(1 << 5)  /* Hibernate reset */
#define RTC_HWRSR_PPR		(1 << 4)  /* PPR reset */
#define RTC_HWRSR_PIN		(1 << 1)  /* Wakeup pin status bit */
#define RTC_HWRSR_ALM		(1 << 0)  /* RTC alarm status bit */

/*
 * RTC Operations
 */
#ifndef __MIPS_ASSEMBLER
#define __rtc_write_ready()		((REG_RTC_RCR & RTC_RCR_WRDY) >> RTC_RCR_WRDY_BIT)
#define __rtc_enabled()			(REG_RTC_RCR |= RTC_RCR_RTCE)
#define __rtc_disabled()		(REG_RTC_RCR &= ~RTC_RCR_RTCE)
#define __rtc_enable_alarm()		(REG_RTC_RCR |= RTC_RCR_AE)
#define __rtc_disable_alarm()		(REG_RTC_RCR &= ~RTC_RCR_AE)
#define __rtc_enable_alarm_irq()	(REG_RTC_RCR |= RTC_RCR_AIE)
#define __rtc_disable_alarm_irq()	(REG_RTC_RCR &= ~RTC_RCR_AIE)
#define __rtc_enable_1Hz_irq()		(REG_RTC_RCR |= RTC_RCR_1HZIE)
#define __rtc_disable_1Hz_irq()		(REG_RTC_RCR &= ~RTC_RCR_1HZIE)

#define __rtc_get_1Hz_flag()		((REG_RTC_RCR >> RTC_RCR_1HZ_BIT) & 0x1)
#define __rtc_clear_1Hz_flag()		(REG_RTC_RCR &= ~RTC_RCR_1HZ)
#define __rtc_get_alarm_flag()		((REG_RTC_RCR >> RTC_RCR_AF_BIT) & 0x1)
#define __rtc_clear_alarm_flag()	(REG_RTC_RCR &= ~RTC_RCR_AF)

#define __rtc_get_second()		(REG_RTC_RSR)
#define __rtc_set_second(v)		(REG_RTC_RSR = (v))

#define __rtc_get_alarm_second()	(REG_RTC_RSAR)
#define __rtc_set_alarm_second(v)	(REG_RTC_RSAR = (v))

#define __rtc_RGR_is_locked()		((REG_RTC_RGR >> RTC_RGR_LOCK))
#define __rtc_lock_RGR()		(REG_RTC_RGR |= RTC_RGR_LOCK)
#define __rtc_unlock_RGR()		(REG_RTC_RGR &= ~RTC_RGR_LOCK)
#define __rtc_get_adjc_val()		((REG_RTC_RGR & RTC_RGR_ADJC_MASK) >> RTC_RGR_ADJC_BIT)
#define __rtc_set_adjc_val(v)	\
       (REG_RTC_RGR = ((REG_RTC_RGR & ~RTC_RGR_ADJC_MASK) | ((v) << RTC_RGR_ADJC_BIT)))
#define __rtc_get_nc1Hz_val()		((REG_RTC_RGR & RTC_RGR_NC1HZ_MASK) >> RTC_RGR_NC1HZ_BIT)
#define __rtc_set_nc1Hz_val(v)	\
       (REG_RTC_RGR = ((REG_RTC_RGR & ~RTC_RGR_NC1HZ_MASK) | ((v) << RTC_RGR_NC1HZ_BIT)))

#define __rtc_power_down()		(REG_RTC_HCR |= RTC_HCR_PD)

#define __rtc_get_hwfcr_val()		(REG_RTC_HWFCR & RTC_HWFCR_MASK)
#define __rtc_set_hwfcr_val(v)		(REG_RTC_HWFCR = (v) & RTC_HWFCR_MASK)
#define __rtc_get_hrcr_val()		(REG_RTC_HRCR & RTC_HRCR_MASK)
#define __rtc_set_hrcr_val(v)		(REG_RTC_HRCR = (v) & RTC_HRCR_MASK)

#define __rtc_enable_alarm_wakeup()	(REG_RTC_HWCR |= RTC_HWCR_EALM)
#define __rtc_disable_alarm_wakeup()	(REG_RTC_HWCR &= ~RTC_HWCR_EALM)

#define __rtc_status_hib_reset_occur()	((REG_RTC_HWRSR >> RTC_HWRSR_HR) & 0x1)
#define __rtc_status_ppr_reset_occur()	((REG_RTC_HWRSR >> RTC_HWRSR_PPR) & 0x1)
#define __rtc_status_wakeup_pin_waken_up()	((REG_RTC_HWRSR >> RTC_HWRSR_PIN) & 0x1)
#define __rtc_status_alarm_waken_up()	((REG_RTC_HWRSR >> RTC_HWRSR_ALM) & 0x1)
#define __rtc_clear_hib_stat_all()	(REG_RTC_HWRSR = 0)

#define __rtc_get_scratch_pattern()	(REG_RTC_HSPR)
#define __rtc_set_scratch_pattern(n)	(REG_RTC_HSPR = (n))

#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4750LRTC_H__ */
