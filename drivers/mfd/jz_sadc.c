#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <linux/jz_sadc.h>
#include <linux/sysrq.h>

#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/jzsoc.h>

#define TS_NAME "jz-sdac"

#define BAT_INTV 30
#define BAT_FULL 4500
#define BAT_LOW  3500
#define BAT_OFF  3350

static unsigned int battery_mv_usb;
static unsigned int battery_low;
static unsigned int battery_mv;
static unsigned int usb_old_state;

extern void fg0_enable(bool);

void sadc_init_clock(void)
{
	unsigned int val;
	int div;
	div = 120 - 1;	/* working at 200 KHz */
	val = div  | (1 << 8) | (199 << 16);
	printk("val = 0x%x\n",val);
	OUTREG32(SADC_ADCLK, val);
	//CMSREG32(SADC_ADCLK, div, ADCLK_CLKDIV_MASK);
	//*(unsigned int *)0xb0070028 = val;

}
/************************************************************************/
/*	 sadc interrupt							*/
/************************************************************************/

struct sadc_sub_irq{
	int irq;
	irqreturn_t (*func)(int irq, void * dev_id);
	void * data;
};


struct sadc_sub_irq sadc_sub_irq[MAX_NUM_IRQ];
void init_sadc_sub_irq(void){
	int i;
	for(i=0;i<MAX_NUM_IRQ;++i){
		sadc_sub_irq[i].irq = -1;
		sadc_sub_irq[i].func = NULL;
		sadc_sub_irq[i].data = NULL;
	}
}

int sadc_request_irq(unsigned int irq,irqreturn_t (*func)(int irq, void * dev_id),void *data){
	if(sadc_sub_irq[irq].func  == NULL){
		sadc_sub_irq[irq].irq = irq;
		sadc_sub_irq[irq].func = func;
		sadc_sub_irq[irq].data = data;
		return 0; //success
	}
	return -1; //fail
}
int sadc_free_irq(unsigned int irq){
	if(sadc_sub_irq[irq].func != NULL){
		sadc_sub_irq[irq].irq = -1;
		sadc_sub_irq[irq].func = NULL;
		sadc_sub_irq[irq].data = NULL;
		return 0; //success
	}
		return -1; //fail
}

static inline void sadc_start_pbat(void)
{
	SETREG8(SADC_ADENA, ADENA_VBATEN);      /* Enable pbat adc */
}

unsigned int jz_read_battery1(void)
{
	unsigned int timeout = 0x3fff;
	u16 pbat;

	sadc_start_pbat();
	udelay(300);

	while(!(INREG8(SADC_ADSTATE) & ADSTATE_VRDY) && --timeout);

	if (!timeout){
		printk(KERN_ERR "Reading battery timeout!");
		return 0;
	}

	pbat = INREG16(SADC_ADVDAT) & ADVDAT_VDATA_MASK;

	OUTREG8(SADC_ADSTATE, ADSTATE_VRDY);
	CLRREG8(SADC_ADENA, ADENA_VBATEN); // hardware may not shut down really

	return pbat;
}

/*
 * Device file operations
 */
static int sadc_open(struct inode *inode, struct file *filp);
static int sadc_release(struct inode *inode, struct file *filp);
static int sadc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations sadc_fops =
{
	open:		sadc_open,
	release:	sadc_release,
	ioctl:		sadc_ioctl
};

static int sadc_open(struct inode *inode, struct file *filp)
{
	try_module_get(THIS_MODULE);
	return 0;
}

static int sadc_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	return 0;
}

static int sadc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	default:
		printk("Not supported command: 0x%x\n", cmd);
		return -EINVAL;
		break;
	}
	return 0;
}

/*
 * procfs interface file operations
 */

static int jz_battery_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	battery_low = simple_strtoul(buffer, 0, 10);
	return count;
}

static int jz_battery_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	//unsigned int mv = (battery_mv-180)/4;// = (jz4740_read_battery() * 7500 + 2048) / 4096;
	unsigned int mv = battery_mv;
	unsigned int value = 0;
	unsigned int usb_state = __gpio_get_pin(GPIO_USB_DETE);

	if (usb_state != usb_old_state) {
		if (usb_state == 0) {
			value = jz_read_battery1();
			if (value != 0){
				mv = (value*2500/4096)*4;

				battery_mv = mv;
			//	printk("usb canel mv is %d\n\n",mv);
			}
		}
		usb_old_state = usb_state;
	}

	if(__gpio_get_pin(GPIO_USB_DETE)) {
		if (mv > battery_mv_usb)
			battery_mv_usb = mv;
		else
			mv = battery_mv_usb;

		mv |= 0x80000000;
	} else {
		if (mv > battery_mv_usb)
			mv = battery_mv_usb;
		else
			battery_mv_usb = mv;

		#ifndef CHARGE_DET
			if(mv > BAT_FULL)
				mv = BAT_FULL;
		#endif
	}

	#ifdef CHARGE_DET
		__gpio_as_input(CHARGE_DET);
		__gpio_disable_pull(CHARGE_DET);
		if(__gpio_get_pin(CHARGE_DET))
			mv = BAT_FULL;
	#endif

	//udc_pnp_set_gpio();

	return sprintf(page, "%u\n", mv);
}

static struct task_struct * battery_monitor;

static void battery_track_timer(unsigned long data)
{
	//printk("%s %d \n",__FILE__,__LINE__);

	int over_time = 0;
	unsigned int value = 0;
	unsigned int mv;

	while (1) {
		value = jz_read_battery1();
		if (value != 0)
			battery_mv = value;

		mv = (battery_mv * 2500 / 4096) * 4; //battery_mv*4+250;
		battery_mv = mv;

		if (!battery_mv_usb) {
			battery_mv_usb = mv;
			printk("Initial battery: %d mv\n", battery_mv_usb);
		}

		//printk("battery is %u \n",mv);

		if (battery_mv < BAT_OFF) {
			over_time++;
			if (over_time > 1) { //3
				printk("Low battery: %d mv\n", battery_mv);
				if (!__gpio_get_pin(OTG_HOTPLUG_PIN)) {
					printk("Battery is too low. Power off...\n");
					// run_sbin_poweroff();
					// jz_pm_hibernate();
					handle_sysrq('s', NULL); /* Force sync */
					handle_sysrq('u', NULL); /* Force read-only remount */
					handle_sysrq('o', NULL); /* Immediate poweroff */
				}
			}
		} else {
			over_time = 0;

			#ifdef BATTERY_LOW_LED
				if (battery_mv < battery_low) {
					fg0_enable(true);
					__gpio_as_output(BATTERY_LOW_LED);
					__gpio_set_pin(BATTERY_LOW_LED);
				}
				if (battery_mv > (battery_low + 20)) {
					fg0_enable(false);
					__gpio_as_output(BATTERY_LOW_LED);
					__gpio_clear_pin(BATTERY_LOW_LED);
				}
			#endif
		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ * BAT_INTV); //10
	}
}

#if 0
irqreturn_t sadc_interrupt(int irq, void * dev_id)
{
	unsigned int state;
	irqreturn_t (*func)(int irq, void * dev_id);
	state = INREG8(SADC_ADSTATE) & (~INREG8(SADC_ADCTRL));

	if(state & ADSTATE_PENU){
		func =sadc_sub_irq[TS_PENUP_IRQ].func;
		if(func != NULL){
			return func(TS_PENUP_IRQ,sadc_sub_irq[TS_PENUP_IRQ].data);
		}
	}else if(state & ADSTATE_PEND){
		func =sadc_sub_irq[TS_PENDOWN_IRQ].func;
		if(func != NULL){
			return func(TS_PENDOWN_IRQ,sadc_sub_irq[TS_PENDOWN_IRQ].data);
		}
	}else if(state & ADSTATE_DTCH){
		func =sadc_sub_irq[TS_DATA_READY_IRQ].func;
		if(func != NULL){
			return func(TS_DATA_READY_IRQ,sadc_sub_irq[TS_DATA_READY_IRQ].data);
		}
	}else if(state & ADSTATE_SLPEND){
		func =sadc_sub_irq[TS_SLPEND_IRQ].func;
		if(func != NULL){
			return func(TS_SLPEND_IRQ,sadc_sub_irq[TS_SLPEND_IRQ].data);
		}
	}else if(state & ADSTATE_VRDY){
		func =sadc_sub_irq[BAT_DATA_READY_IRQ].func;
		if(func != NULL){
			return func(BAT_DATA_READY_IRQ,sadc_sub_irq[BAT_DATA_READY_IRQ].data);
		}
	}
	return IRQ_HANDLED;
}
#endif

/************************************************************************/
/*	init sadc							*/
/************************************************************************/
static int __init sadc_init(void)
{
	struct proc_dir_entry *res;
	int	error;
	cpm_start_clock(CGM_SADC); //start sadc clock
	udelay(1);

#if defined(CONFIG_SOC_JZ4760)
	CLRREG8(SADC_ADENA, ADENA_POWER);
	CLRREG32(CPM_LCR, LCR_VBATIR);
	mdelay(50);
#elif defined(CONFIG_SOC_JZ4760B) || defined(CONFIG_SOC_JZ4770)
	CLRREG8(SADC_ADENA, ADENA_POWER); //sadc power on
	mdelay(50); //wait sadc power on
	SETREG8(SADC_ADENA, ADENA_PENDEN); //enable pen down detect
#endif

	sadc_init_clock(); //init sadc clock
	OUTREG8(SADC_ADCTRL, ADCTRL_MASK_ALL); //mask all interrupt

	usb_old_state = 0;
	battery_mv_usb = 0;
	battery_low = BAT_LOW;

#if defined(CONFIG_SOC_JZ4760B) && defined(CONFIG_JZ4760_LCD_RG_IPS)
	battery_low = BAT_LOW - 100;
#endif

	battery_mv = BAT_FULL;

	res = create_proc_entry("jz/battery", 0, NULL);
	if (res) {
		res->read_proc = jz_battery_read_proc;
		res->write_proc = jz_battery_write_proc;
	}

	battery_monitor = kthread_run(battery_track_timer, NULL, "jz/battery");

	if (IS_ERR(battery_monitor)) {
	  printk("Kernel battery_monitor thread start error!\n");
	  return 0;
	}

	printk("JZ4760b SAR-ADC driver registered\n");
	return 0;
}

static void __exit sadc_exit(void)
{
	remove_proc_entry("jz/battery", NULL);
}
//subsys_initcall(sadc_init);
module_init(sadc_init);
module_exit(sadc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ SADC Core Driver");
MODULE_AUTHOR(" <@ingenic.com>");
