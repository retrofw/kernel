#ifndef JZ_SADC_H
#define JZ_SADC_H

#include <linux/interrupt.h>
#include <asm/irq.h>

#define TS_PENUP_IRQ  0
#define TS_PENDOWN_IRQ 1
#define TS_DATA_READY_IRQ  2
#define TS_SLPEND_IRQ  3
#define BAT_DATA_READY_IRQ 4

#define MAX_NUM_IRQ 5
/************************************************************************/
/*	SAR ADC  BATTERY  INTERFACE							*/
/* read:1,enable interrupt*/
/*          2,start sample*/
/*          3,once data ready interrupt product auto and sample stop auto by hardware */
/*          4,read data*/
/*          5,when you don't want to read data ,disable the  interrupt*/
/************************************************************************/
/*inline void sadc_start_pbat(void);                                          //start sample battery data
inline void sadc_stop_pbat(void);                                           //stop sample battery data
inline void disable_battery_data_ready_irq(void);
inline void enable_battery_data_ready_irq(void);   // only battery_data_ready_interrupt enable ,when sample ready , can battery_data_ready_interrupt product
inline void clean_battery_data_ready_irq_state(void);
u16 get_pbat_after_data_ready_irq(void);
unsigned int jz_read_battery_p0(void);
*/
/************************************************************************/
/*	SAR ADC  TOUCHSCREEN  INTERFACE							*/
/************************************************************************/
/*void sadc_start_ts(void);
inline void sadc_enable_ts(void);
inline void sadc_disable_ts(void);
inline void ts_enable_pendown_irq(void);
inline void ts_disable_pendown_irq(void);
inline void ts_enable_penup_irq(void);
inline void ts_disable_penup_irq(void);
inline void ts_enable_data_ready_irq(void);
inline void ts_disable_data_ready_irq(void);
inline void clean_penup_irq_state(void);
inline void clean_pendown_irq_state(void);
inline void clean_ts_data_ready_irq_state(void);
void read_touchscreen_data_after_data_ready_irq(unsigned int fifo[]);
inline void ts_disable_sleep_pendown_irq(void);
*/
/************************************************************************/
/*	SAR ADC  AUX  FOR KEY INTERFACE							*/
/************************************************************************/
/*inline void sadc_start_aux(void);
unsigned short jz_read_aux(int index);
*/
/************************************************************************/
/*	REQUEST IRQ  INTERFACE							*/
/************************************************************************/
void sadc_init_clock(void);
int sadc_request_irq(unsigned int irq,irqreturn_t(*func)(int irq, void * dev_id),void *data);
int sadc_free_irq(unsigned int irq);

#endif
