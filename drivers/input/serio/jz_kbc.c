/*
 *  jz_kbc keyboard and mouse controller driver for Linux
 *
 *  Copyright (c) 1999-2004 Vojtech Pavlik
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/err.h>
#include <linux/rcupdate.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/jzsoc.h>

MODULE_DESCRIPTION("ingenic keyboard and mouse controller driver");
MODULE_LICENSE("GPL");

static unsigned int jz_kbc_nokbd;
module_param_named(nokbd, jz_kbc_nokbd, bool, 0);
MODULE_PARM_DESC(nokbd, "Do not probe or use KBD port.");

static unsigned int jz_kbc_noaux;
module_param_named(noaux, jz_kbc_noaux, bool, 0);
MODULE_PARM_DESC(noaux, "Do not probe or use AUX (mouse) port.");

#if 0
static unsigned int jz_kbc_nomux;
module_param_named(nomux, jz_kbc_nomux, bool, 0);
MODULE_PARM_DESC(nomux, "Do not check whether an active multiplexing conrtoller is present.");

module_param_named(unlock, jz_kbc_unlock, bool, 0);
MODULE_PARM_DESC(unlock, "Ignore keyboard lock.");
#endif

static unsigned int jz_kbc_direct;
module_param_named(direct, jz_kbc_direct, bool, 0);
MODULE_PARM_DESC(direct, "Put keyboard port into non-translated mode.");

static unsigned int jz_kbc_noloop;
module_param_named(noloop, jz_kbc_noloop, bool, 0);
MODULE_PARM_DESC(noloop, "Disable the AUX Loopback command while probing for the AUX port");

static unsigned int jz_kbc_blink_frequency = 500;
module_param_named(panicblink, jz_kbc_blink_frequency, uint, 0600);
MODULE_PARM_DESC(panicblink, "Frequency with which keyboard LEDs should blink when kernel panics");

#define DEBUG
#ifdef DEBUG
static int jz_kbc_debug;
module_param_named(debug, jz_kbc_debug, bool, 0600);
MODULE_PARM_DESC(debug, "Turn jz_kbc debugging mode on and off");
#endif

#include "jz_kbc.h"

static DEFINE_SPINLOCK(jz_kbc_lock);

struct jz_kbc_port {
	struct serio *serio;
	int irq;
	unsigned char exists;
	signed char mux;
};

#define KBC_KBD_PORT_NO	0
#define KBC_AUX_PORT_NO	1
//#define KBC_MUX_PORT_NO	2
//#define KBC_NUM_PORTS		(KBC_NUM_MUX_PORTS + 2)
#define KBC_NUM_PORTS		2 

static struct jz_kbc_port jz_kbc_ports[KBC_NUM_PORTS];

static unsigned char jz_kbc_initial_ctr;
static unsigned char jz_kbc_ctr;
//static unsigned char jz_kbc_mux_present;
static unsigned char jz_kbc_suppress_kbd_ack;
static struct platform_device *jz_kbc_platform_device;

static irqreturn_t jz_kbc_interrupt(int irq, void *dev_id);

/*
 * The jz_kbc_wait_read() and jz_kbc_wait_write functions wait for the jz_kbc to
 * be ready for reading values from it / writing values to it.
 * Called always with jz_kbc_lock held.
 * jz_kbc_read_status() achieve in jz_kbc-****.h 
 */

static int jz_kbc_wait_read(void)
{
	int i = 0;

	while ((~jz_kbc_read_status() & KBC_STR_OBF) && (i < KBC_CTL_TIMEOUT)) {
		udelay(50);
		i++;
	}
	return -(i == KBC_CTL_TIMEOUT);
}

static int jz_kbc_wait_write(void)
{
	int i = 0;

	while ((jz_kbc_read_status() & KBC_STR_IBF) && (i < KBC_CTL_TIMEOUT)) {
		udelay(50);
		i++;
	}
	return -(i == KBC_CTL_TIMEOUT);
}

/*
 * jz_kbc_flush() flushes all data that may be in the keyboard and mouse buffers
 * of the jz_kbc down the toilet.
 */
static int jz_kbc_flush(void)
{
	unsigned long flags;
	unsigned char data, str;
	int i = 0;

	spin_lock_irqsave(&jz_kbc_lock, flags);

	while (((str = jz_kbc_read_status()) & KBC_STR_OBF) && (i < KBC_BUFFER_SIZE)) {
		udelay(50);
		data = jz_kbc_read_data();
		i++;
		dbg("%02x <- jz_kbc (flush, %s)", data,
			str & KBC_STR_AUXDATA ? "aux" : "kbd");
	}

	spin_unlock_irqrestore(&jz_kbc_lock, flags);

	return i;
}

/*
 * jz_kbc_command() executes a command on the jz_kbc. It also sends the input
 * parameter(s) of the commands to it, and receives the output value(s). The
 * parameters are to be stored in the param array, and the output is placed
 * into the same array. The number of the parameters and output values is
 * encoded in bits 8-11 of the command number.		   ACK len 
 * bit12: 0 - none parameter 0byte; 1 - 8byt eparameter
 */

static int __jz_kbc_command(unsigned char *param, int command)
{
	int i, error;

	if (jz_kbc_noloop && command == KBC_CMD_AUX_LOOP)
		return -1;

	error = jz_kbc_wait_write();
	if (error)
		return error;

	dbg("%02x -> jz_kbc (command)", command & 0xff);
	jz_kbc_write_command(command & 0xff);

	for (i = 0; i < ((command >> 12) & 0xf); i++) {
		error = jz_kbc_wait_write();
		if (error)
			return error;
		dbg("%02x -> jz_kbc (parameter)", param[i]);
		jz_kbc_write_data(param[i]);
	}

	for (i = 0; i < ((command >> 8) & 0xf); i++) {
		error = jz_kbc_wait_read();
		if (error) {
			dbg("     -- jz_kbc (timeout)");
			return error;
		}

		if (command == KBC_CMD_AUX_LOOP &&
		    !(jz_kbc_read_status() & KBC_STR_AUXDATA)) {
			dbg("     -- jz_kbc (auxerr)");
			return -1;
		}
		param[i] = jz_kbc_read_data();
		dbg("%02x <- jz_kbc (return)", param[i]);
	}

	return 0;
}

int jz_kbc_command(unsigned char *param, int command)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&jz_kbc_lock, flags);
	retval = __jz_kbc_command(param, command);
	spin_unlock_irqrestore(&jz_kbc_lock, flags);

	return retval;
}
EXPORT_SYMBOL(jz_kbc_command);

/*
 * jz_kbc_kbd_write() sends a byte out through the keyboard interface.
 */

static int jz_kbc_kbd_write(struct serio *port, unsigned char c)
{
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&jz_kbc_lock, flags);

	if (!(retval = jz_kbc_wait_write())) {
		dbg("%02x -> jz_kbc (kbd-data)", c);
		jz_kbc_write_data(c);
	}

	spin_unlock_irqrestore(&jz_kbc_lock, flags);

	return retval;
}

/*
 * jz_kbc_aux_write() sends a byte out through the aux interface.
 */

static int jz_kbc_aux_write(struct serio *serio, unsigned char c)
{
#if 0
	struct jz_kbc_port *port = serio->port_data;

	return jz_kbc_command(&c, port->mux == -1 ?
					KBC_CMD_AUX_SEND :
					KBC_CMD_MUX_SEND + port->mux);
#endif
	return jz_kbc_command(&c, KBC_CMD_AUX_SEND);
}

/*
 * jz_kbc_start() is called by serio core when port is about to finish
 * registering. It will mark port as existing so jz_kbc_interrupt can
 * start sending data through it.
 */
static int jz_kbc_start(struct serio *serio)
{
	struct jz_kbc_port *port = serio->port_data;

	port->exists = 1;
	mb();
	return 0;
}

/*
 * jz_kbc_stop() marks serio port as non-existing so jz_kbc_interrupt
 * will not try to send data to the port that is about to go away.
 * The function is called by serio core as part of unregister procedure.
 */
static void jz_kbc_stop(struct serio *serio)
{
	struct jz_kbc_port *port = serio->port_data;

	port->exists = 0;

	/*
	 * We synchronize with both AUX and KBD IRQs because there is
	 * a (very unlikely) chance that AUX IRQ is raised for KBD port
	 * and vice versa.
	 */
	synchronize_irq(IRQ_KBC);
	port->serio = NULL;
}

/*
 * jz_kbc_interrupt() is the most important function in this driver -
 * it handles the interrupts from the jz_kbc, and sends incoming bytes
 * to the upper layers.
 */

static irqreturn_t jz_kbc_interrupt(int irq, void *dev_id)
{
	struct jz_kbc_port *port;
	unsigned long flags;
	unsigned char str, data;
	unsigned int dfl;
	unsigned int port_no;
	int ret = 1;

	spin_lock_irqsave(&jz_kbc_lock, flags);
	str = jz_kbc_read_status();
	if (unlikely(~str & KBC_STR_OBF)) {
		spin_unlock_irqrestore(&jz_kbc_lock, flags);
		if (irq) dbg("Interrupt %d, without any data", irq);
		ret = 0;
		goto out;
	}
	data = jz_kbc_read_data();
	spin_unlock_irqrestore(&jz_kbc_lock, flags);

#if 0
	if (jz_kbc_mux_present && (str & KBC_STR_AUXDATA)) {
		static unsigned long last_transmit;
		static unsigned char last_str;

		dfl = 0;
		if (str & KBC_STR_MUXERR) {
			dbg("MUX error, status is %02x, data is %02x", str, data);
/*
 * When MUXERR condition is signalled the data register can only contain
 * 0xfd, 0xfe or 0xff if implementation follows the spec. Unfortunately
 * it is not always the case. Some KBCs also report 0xfc when there is
 * nothing connected to the port while others sometimes get confused which
 * port the data came from and signal error leaving the data intact. They
 * _do not_ revert to legacy mode (actually I've never seen KBC reverting
 * to legacy mode yet, when we see one we'll add proper handling).
 * Anyway, we process 0xfc, 0xfd, 0xfe and 0xff as timeouts, and for the
 * rest assume that the data came from the same serio last byte
 * was transmitted (if transmission happened not too long ago).
 */

			switch (data) {
				default:
					if (time_before(jiffies, last_transmit + HZ/10)) {
						str = last_str;
						break;
					}
					/* fall through - report timeout */
				case 0xfc:
				case 0xfd:
				case 0xfe: dfl = SERIO_TIMEOUT; data = 0xfe; break;
				case 0xff: dfl = SERIO_PARITY;  data = 0xfe; break;
			}
		}

		port_no = KBC_MUX_PORT_NO + ((str >> 6) & 3);
		last_str = str;
		last_transmit = jiffies;
	} else {
#endif

		dfl = ((str & KBC_STR_PARITY) ? SERIO_PARITY : 0) |
		      ((str & KBC_STR_TIMEOUT) ? SERIO_TIMEOUT : 0);

		port_no = (str & KBC_STR_AUXDATA) ?
				KBC_AUX_PORT_NO : KBC_KBD_PORT_NO;
	//}

	port = &jz_kbc_ports[port_no];

	dbg("%02x <- jz_kbc (interrupt, %d, %d%s%s)",
	    data, port_no, irq,
	    dfl & SERIO_PARITY ? ", bad parity" : "",
	    dfl & SERIO_TIMEOUT ? ", timeout" : "");

	if (unlikely(jz_kbc_suppress_kbd_ack))
		if (port_no == KBC_KBD_PORT_NO &&
		    (data == 0xfa || data == 0xfe)) {
			jz_kbc_suppress_kbd_ack--;
			goto out;
		}

	if (likely(port->exists))
		serio_interrupt(port->serio, data, dfl);

 out:
	return IRQ_RETVAL(ret);
}

/*
 * jz_kbc_enable_kbd_port enables keybaord port on chip
 */

static int jz_kbc_enable_kbd_port(void)
{
	jz_kbc_ctr &= ~KBC_CTR_KBDDIS;
	jz_kbc_ctr |= KBC_CTR_KBDINT;

	if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_WCTR)) {
		jz_kbc_ctr &= ~KBC_CTR_KBDINT;
		jz_kbc_ctr |= KBC_CTR_KBDDIS;
		printk(KERN_ERR "jz_kbc.c: Failed to enable KBD port.\n");
		return -EIO;
	}

	return 0;
}

/*
 * jz_kbc_enable_aux_port enables AUX (mouse) port on chip
 */

static int jz_kbc_enable_aux_port(void)
{
	jz_kbc_ctr &= ~KBC_CTR_AUXDIS;
	jz_kbc_ctr |= KBC_CTR_AUXINT;

	if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_WCTR)) {
		jz_kbc_ctr &= ~KBC_CTR_AUXINT;
		jz_kbc_ctr |= KBC_CTR_AUXDIS;
		printk(KERN_ERR "jz_kbc.c: Failed to enable AUX port.\n");
		return -EIO;
	}

	return 0;
}

/*
 * jz_kbc_enable_mux_ports enables 4 individual AUX ports after
 * the controller has been switched into Multiplexed mode
 */

#if 0
static int jz_kbc_enable_mux_ports(void)
{
	unsigned char param;
	int i;

	for (i = 0; i < KBC_NUM_MUX_PORTS; i++) {
		jz_kbc_command(&param, KBC_CMD_MUX_PFX + i);
		jz_kbc_command(&param, KBC_CMD_AUX_ENABLE);
	}

	return jz_kbc_enable_aux_port();
}

/*
 * jz_kbc_set_mux_mode checks whether the controller has an active
 * multiplexor and puts the chip into Multiplexed (1) or Legacy (0) mode.
 */

static int jz_kbc_set_mux_mode(unsigned int mode, unsigned char *mux_version)
{

	unsigned char param;
/*
 * Get rid of bytes in the queue.
 */

//jz_kbc_flush();

/*
 * Internal loopback test - send three bytes, they should come back from the
 * mouse interface, the last should be version.
 */

	param = 0xf0;
	if (jz_kbc_command(&param, KBC_CMD_AUX_LOOP) || param != 0xf0)
		return -1;
	param = mode ? 0x56 : 0xf6;
	if (jz_kbc_command(&param, KBC_CMD_AUX_LOOP) || param != (mode ? 0x56 : 0xf6))
		return -1;
	param = mode ? 0xa4 : 0xa5;
	if (jz_kbc_command(&param, KBC_CMD_AUX_LOOP) || param == (mode ? 0xa4 : 0xa5))
		return -1;

	if (mux_version)
		*mux_version = param;

	return 0;
}

/*
 * jz_kbc_check_mux() checks whether the controller supports the PS/2 Active
 * Multiplexing specification by Synaptics, Phoenix, Insyde and
 * LCS/Telegraphics.
 */

static int __devinit jz_kbc_check_mux(void)
{
	unsigned char mux_version;

	if (jz_kbc_set_mux_mode(1, &mux_version))
		return -1;

/*
 * Workaround for interference with USB Legacy emulation
 * that causes a v10.12 MUX to be found.
 */
	if (mux_version == 0xAC)
		return -1;

	printk(KERN_INFO "jz_kbc.c: Detected active multiplexing controller, rev %d.%d.\n",
		(mux_version >> 4) & 0xf, mux_version & 0xf);

/*
 * Disable all muxed ports by disabling AUX.
 */
	jz_kbc_ctr |= KBC_CTR_AUXDIS;
	jz_kbc_ctr &= ~KBC_CTR_AUXINT;

	if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_WCTR)) {
		printk(KERN_ERR "jz_kbc.c: Failed to disable AUX port, can't use MUX.\n");
		return -EIO;
	}

	jz_kbc_mux_present = 1;

	return 0;
}
#endif

/*
 * The following is used to test AUX IRQ delivery.
 */
static struct completion jz_kbc_aux_irq_delivered __devinitdata;
static int jz_kbc_irq_being_tested __devinitdata;

static irqreturn_t __devinit jz_kbc_aux_test_irq(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned char str, data;
	int ret = 0;

	spin_lock_irqsave(&jz_kbc_lock, flags);
	str = jz_kbc_read_status();
	if (str & KBC_STR_OBF) {
		data = jz_kbc_read_data();
		if (jz_kbc_irq_being_tested &&
		    data == 0xa5 && (str & KBC_STR_AUXDATA))
			complete(&jz_kbc_aux_irq_delivered);
		ret = 1;
	}
	spin_unlock_irqrestore(&jz_kbc_lock, flags);

	return IRQ_RETVAL(ret);
}

/*
 * jz_kbc_toggle_aux - enables or disables AUX port on jz_kbc via command and
 * verifies success by readinng CTR. Used when testing for presence of AUX
 * port.
 */
static int __devinit jz_kbc_toggle_aux(int on)
{
	unsigned char param;
	int i;

	if (jz_kbc_command(&param,
			on ? KBC_CMD_AUX_ENABLE : KBC_CMD_AUX_DISABLE))
		return -1;

	/* some chips need some time to set the KBC_CTR_AUXDIS bit */
	for (i = 0; i < 100; i++) {
		udelay(50);

		if (jz_kbc_command(&param, KBC_CMD_CTL_RCTR))
			return -1;

		if (!(param & KBC_CTR_AUXDIS) == on)
			return 0;
	}

	return -1;
}

/*
 * jz_kbc_check_aux() applies as much paranoia as it can at detecting
 * the presence of an AUX interface.
 */

static int __devinit jz_kbc_check_aux(void)
{
	int retval = -1;
	int irq_registered = 0;
	int aux_loop_broken = 0;
	unsigned long flags;
	unsigned char param;

/*
 * Get rid of bytes in the queue.
 */

	jz_kbc_flush();

/*
 * Internal loopback test - filters out AT-type jz_kbc's. Unfortunately
 * SiS screwed up and their 5597 doesn't support the LOOP command even
 * though it has an AUX port.
 */

	param = 0x5a;
	retval = jz_kbc_command(&param, KBC_CMD_AUX_LOOP);
	if (retval || param != 0x5a) {

/*
 * External connection test - filters out AT-soldered PS/2 jz_kbc's
 * 0x00 - no error, 0x01-0x03 - clock/data stuck, 0xff - general error
 * 0xfa - no error on some notebooks which ignore the spec
 * Because it's common for chipsets to return error on perfectly functioning
 * AUX ports, we test for this only when the LOOP command failed.
 */

		if (jz_kbc_command(&param, KBC_CMD_AUX_TEST) ||
		    (param && param != 0xfa && param != 0xff))
			return -1;

/*
 * If AUX_LOOP completed without error but returned unexpected data
 * mark it as broken
 */
		if (!retval)
			aux_loop_broken = 1;
	}

/*
 * Bit assignment test - filters out PS/2 jz_kbc's in AT mode
 */

	if (jz_kbc_toggle_aux(0)) {
		printk(KERN_WARNING "Failed to disable AUX port, but continuing anyway... Is this a SiS?\n");
		printk(KERN_WARNING "If AUX port is really absent please use the 'jz_kbc.noaux' option.\n");
	}

	if (jz_kbc_toggle_aux(1))
		return -1;

/*
 * Test AUX IRQ delivery to make sure BIOS did not grab the IRQ and
 * used it for a PCI card or somethig else.
 */

	if (jz_kbc_noloop || aux_loop_broken) {
/*
 * Without LOOP command we can't test AUX IRQ delivery. Assume the port
 * is working and hope we are right.
 */
		retval = 0;
		goto out;
	}
	if (request_irq(IRQ_KBC, jz_kbc_aux_test_irq, IRQF_SHARED,
			"jz_kbc", jz_kbc_platform_device))
		goto out;

	irq_registered = 1;

	if (jz_kbc_enable_aux_port())
		goto out;

	spin_lock_irqsave(&jz_kbc_lock, flags);

	init_completion(&jz_kbc_aux_irq_delivered);
	jz_kbc_irq_being_tested = 1;

	param = 0xa5;
	retval = __jz_kbc_command(&param, KBC_CMD_AUX_LOOP & 0xf0ff);

	spin_unlock_irqrestore(&jz_kbc_lock, flags);

	if (retval)
		goto out;

	if (wait_for_completion_timeout(&jz_kbc_aux_irq_delivered,
					msecs_to_jiffies(250)) == 0) {
/*
 * AUX IRQ was never delivered so we need to flush the controller to
 * get rid of the byte we put there; otherwise keyboard may not work.
 */
		jz_kbc_flush();
		retval = -1;
	}

//printk(KERN_INFO "jz_bkc_check_aux() ok!!!retval = %d\n", retval);
 out:

/*
 * Disable the interface.
 */

	jz_kbc_ctr |= KBC_CTR_AUXDIS;
	jz_kbc_ctr &= ~KBC_CTR_AUXINT;

	if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_WCTR))
		retval = -1;

	if (irq_registered)
		free_irq(IRQ_KBC, jz_kbc_platform_device);

	return retval;
}

static int jz_kbc_controller_check(void)
{
	if (jz_kbc_flush() == KBC_BUFFER_SIZE) {
		printk(KERN_ERR "jz_kbc.c: No controller found.\n");
		return -ENODEV;
	}

	return 0;
}

static int jz_kbc_controller_selftest(void)
{
	unsigned char param;
	int i = 0;

	/*
	 * We try this 5 times; on some really fragile systems this does not
	 * take the first time...
	 */
	do {

		if (jz_kbc_command(&param, KBC_CMD_CTL_TEST)) {
			printk(KERN_ERR "jz_kbc.c: jz_kbc controller self test timeout.\n");
			return -ENODEV;
		}

		if (param == KBC_RET_CTL_TEST)
			return 0;

		printk(KERN_ERR "jz_kbc.c: jz_kbc controller selftest failed. (%#x != %#x)\n",
			param, KBC_RET_CTL_TEST);
		msleep(50);
	} while (i++ < 5);

	return -EIO;
}

/*
 * jz_kbc_controller init initializes the jz_kbc controller, and,
 * most importantly, sets it into non-xlated mode if that's
 * desired.
 */

static int jz_kbc_controller_init(void)
{
	//unsigned long flags;

/*
 * Save the CTR for restoral on unload / reboot.
 */

	if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_RCTR)) {
		printk(KERN_ERR "jz_kbc.c: Can't read CTR while initializing jz_kbc.\n");
		return -EIO;
	}

	jz_kbc_initial_ctr = jz_kbc_ctr;

/*
 * Disable the keyboard interface and interrupt.
 */

	jz_kbc_ctr |= KBC_CTR_KBDDIS;
	jz_kbc_ctr &= ~KBC_CTR_KBDINT;

/*
 * Handle keylock.Current KBC donesn't support.
 */
#if 0

	spin_lock_irqsave(&jz_kbc_lock, flags);
	if (~jz_kbc_read_status() & KBC_STR_KEYLOCK) {
		if (jz_kbc_unlock)
			jz_kbc_ctr |= KBC_CTR_IGNKEYLOCK;
		else
			printk(KERN_WARNING "jz_kbc.c: Warning: Keylock active.\n");
	}
	spin_unlock_irqrestore(&jz_kbc_lock, flags);
#endif

/*
 * If the chip is configured into nontranslated mode by the BIOS, don't
 * bother enabling translating and be happy.
 */

	if (~jz_kbc_ctr & KBC_CTR_XLATE)
		jz_kbc_direct = 1;

/*
 * Set nontranslated mode for the kbd interface if requested by an option.
 * After this the kbd interface becomes a simple serial in/out, like the aux
 * interface is. We don't do this by default, since it can confuse notebook
 * BIOSes.
 */

	if (jz_kbc_direct)
		jz_kbc_ctr &= ~KBC_CTR_XLATE;

/*
 * Write CTR back.
 */

	if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_WCTR)) {
		printk(KERN_ERR "jz_kbc.c: Can't write CTR while initializing jz_kbc.\n");
		return -EIO;
	}

	return 0;
}


/*
 * Reset the controller and reset CRT to the original value set by BIOS.
 */

static void jz_kbc_controller_reset(void)
{
	jz_kbc_flush();

/*
 * Disable both KBD and AUX interfaces so they don't get in the way
 */

	jz_kbc_ctr |= KBC_CTR_KBDDIS | KBC_CTR_AUXDIS;
	jz_kbc_ctr &= ~(KBC_CTR_KBDINT | KBC_CTR_AUXINT);

/*
 * Disable MUX mode if present.
 */

#if 0
	if (jz_kbc_mux_present)
		jz_kbc_set_mux_mode(0, NULL);
#endif

/*
 * Reset the controller if requested.
 */

	jz_kbc_controller_selftest();

/*
 * Restore the original control register setting.
 */

	if (jz_kbc_command(&jz_kbc_initial_ctr, KBC_CMD_CTL_WCTR))
		printk(KERN_WARNING "jz_kbc.c: Can't restore CTR.\n");
}


/*
 * jz_kbc_panic_blink() will flash the keyboard LEDs and is called when
 * kernel panics. Flashing LEDs is useful for users running X who may
 * not see the console and will help distingushing panics from "real"
 * lockups.
 *
 * Note that DELAY has a limit of 10ms so we will not get stuck here
 * waiting for KBC to free up even if KBD interrupt is off
 */

#define DELAY do { mdelay(1); if (++delay > 10) return delay; } while(0)

static long jz_kbc_panic_blink(long count)
{
	long delay = 0;
	static long last_blink;
	static char led;

	/*
	 * We expect frequency to be about 1/2s. KDB uses about 1s.
	 * Make sure they are different.
	 */
	if (!jz_kbc_blink_frequency)
		return 0;
	if (count - last_blink < jz_kbc_blink_frequency)
		return 0;

	led ^= 0x01 | 0x04;
	while (jz_kbc_read_status() & KBC_STR_IBF)
		DELAY;
	dbg("%02x -> jz_kbc (panic blink)", 0xed);
	jz_kbc_suppress_kbd_ack = 2;
	jz_kbc_write_data(0xed); /* set leds */
	DELAY;
	while (jz_kbc_read_status() & KBC_STR_IBF)
		DELAY;
	DELAY;
	dbg("%02x -> jz_kbc (panic blink)", led);
	jz_kbc_write_data(led);
	DELAY;
	last_blink = count;
	return delay;
}

#undef DELAY

#ifdef CONFIG_PM

static bool jz_kbc_suspended;

/*
 * Here we try to restore the original BIOS settings. We only want to
 * do that once, when we really suspend, not when we taking memory
 * snapshot for swsusp (in this case we'll perform required cleanup
 * as part of shutdown process).
 */

static int jz_kbc_suspend(struct platform_device *dev, pm_message_t state)
{
	if (!jz_kbc_suspended && state.event == PM_EVENT_SUSPEND)
		jz_kbc_controller_reset();

	jz_kbc_suspended = state.event == PM_EVENT_SUSPEND ||
			  state.event == PM_EVENT_FREEZE;

	return 0;
}


/*
 * Here we try to reset everything back to a state in which suspended
 */

static int jz_kbc_resume(struct platform_device *dev)
{
	int error;

/*
 * Do not bother with restoring state if we haven't suspened yet
 */
	if (!jz_kbc_suspended)
		return 0;

	error = jz_kbc_controller_check();
	if (error)
		return error;

	error = jz_kbc_controller_selftest();
	if (error)
		return error;

/*
 * Restore original CTR value and disable all ports
 */

	jz_kbc_ctr = jz_kbc_initial_ctr;
	if (jz_kbc_direct)
		jz_kbc_ctr &= ~KBC_CTR_XLATE;
	jz_kbc_ctr |= KBC_CTR_AUXDIS | KBC_CTR_KBDDIS;
	jz_kbc_ctr &= ~(KBC_CTR_AUXINT | KBC_CTR_KBDINT);
	if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_WCTR)) {
		printk(KERN_WARNING "jz_kbc: Can't write CTR to resume, retrying...\n");
		msleep(50);
		if (jz_kbc_command(&jz_kbc_ctr, KBC_CMD_CTL_WCTR)) {
			printk(KERN_ERR "jz_kbc: CTR write retry failed\n");
			return -EIO;
		}
	}


#if 0

	if (jz_kbc_mux_present) {
		if (jz_kbc_set_mux_mode(1, NULL) || jz_kbc_enable_mux_ports())
			printk(KERN_WARNING
				"jz_kbc: failed to resume active multiplexor, "
				"mouse won't work.\n");
	} else 
#endif 
	if (jz_kbc_ports[KBC_AUX_PORT_NO].serio)
		jz_kbc_enable_aux_port();

	if (jz_kbc_ports[KBC_KBD_PORT_NO].serio)
		jz_kbc_enable_kbd_port();

	jz_kbc_suspended = false;
	jz_kbc_interrupt(0, NULL);

	return 0;
}
#endif /* CONFIG_PM */

/*
 * We need to reset the 8042 back to original mode on system shutdown,
 * because otherwise BIOSes will be confused.
 */

static void jz_kbc_shutdown(struct platform_device *dev)
{
	jz_kbc_controller_reset();
}

static int __devinit jz_kbc_create_kbd_port(void)
{
	struct serio *serio;
	struct jz_kbc_port *port = &jz_kbc_ports[KBC_KBD_PORT_NO];

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	serio->id.type		= jz_kbc_direct ? SERIO_8042 : SERIO_8042_XL;
	serio->write		= jz_kbc_kbd_write;
	serio->start		= jz_kbc_start;
	serio->stop		= jz_kbc_stop;
	serio->port_data	= port;
	serio->dev.parent	= &jz_kbc_platform_device->dev;
	strlcpy(serio->name, "jz_kbc KBD port", sizeof(serio->name));
	strlcpy(serio->phys, KBC_KBD_PHYS_DESC, sizeof(serio->phys));

	port->serio = serio;
	port->irq = IRQ_KBC;

	return 0;
}

static int __devinit jz_kbc_create_aux_port(int idx)
{
	struct serio *serio;
//int port_no = idx < 0 ? KBC_AUX_PORT_NO : KBC_MUX_PORT_NO + idx;
//struct jz_kbc_port *port = &jz_kbc_ports[port_no];
	struct jz_kbc_port *port = &jz_kbc_ports[KBC_AUX_PORT_NO];

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	serio->id.type		= SERIO_8042;
	serio->write		= jz_kbc_aux_write;
	serio->start		= jz_kbc_start;
	serio->stop		= jz_kbc_stop;
	serio->port_data	= port;
	serio->dev.parent	= &jz_kbc_platform_device->dev;
	if (idx < 0) {
		strlcpy(serio->name, "jz_kbc AUX port", sizeof(serio->name));
		strlcpy(serio->phys, KBC_AUX_PHYS_DESC, sizeof(serio->phys));
	} else {
		snprintf(serio->name, sizeof(serio->name), "jz_kbc AUX%d port", idx);
		snprintf(serio->phys, sizeof(serio->phys), KBC_MUX_PHYS_DESC, idx + 1);
	}

	port->serio = serio;
	port->mux = idx;
	port->irq = IRQ_KBC;

	return 0;
}

static void __devinit jz_kbc_free_kbd_port(void)
{
	kfree(jz_kbc_ports[KBC_KBD_PORT_NO].serio);
	jz_kbc_ports[KBC_KBD_PORT_NO].serio = NULL;
}

static void __devinit jz_kbc_free_aux_ports(void)
{
	int i;

	for (i = KBC_AUX_PORT_NO; i < KBC_NUM_PORTS; i++) {
		kfree(jz_kbc_ports[i].serio);
		jz_kbc_ports[i].serio = NULL;
	}
}

static void __devinit jz_kbc_register_ports(void)
{
	int i;

	for (i = 0; i < KBC_NUM_PORTS; i++) {
		if (jz_kbc_ports[i].serio) {
			printk(KERN_INFO "serio: %s at %#lx,%#lx irq %d\n",
					jz_kbc_ports[i].serio->name,
					(unsigned long) KBC_DATA_REG,
					(unsigned long) KBC_COMMAND_REG,
					jz_kbc_ports[i].irq);
			serio_register_port(jz_kbc_ports[i].serio);
		}
	}
}

static void __devexit jz_kbc_unregister_ports(void)
{
	int i;

	for (i = 0; i < KBC_NUM_PORTS; i++) {
		if (jz_kbc_ports[i].serio) {
			serio_unregister_port(jz_kbc_ports[i].serio);
			jz_kbc_ports[i].serio = NULL;
		}
	}
}

static int __devinit jz_kbc_setup_aux(void)
{
	int (*aux_enable)(void);
	int error;
	//int i;

	printk(KERN_INFO "jz_bkc_setup_aux()\n");

	if (jz_kbc_check_aux())
		return -ENODEV;

	error = jz_kbc_create_aux_port(-1);
#if 0
	if (jz_kbc_nomux || jz_kbc_check_mux()) {
		error = jz_kbc_create_aux_port(-1);
		if (error)
			goto err_free_ports;
		aux_enable = jz_kbc_enable_aux_port;
	} else {
		for (i = 0; i < KBC_NUM_MUX_PORTS; i++) {
			error = jz_kbc_create_aux_port(i);
			if (error)
				goto err_free_ports;
		}
		aux_enable = jz_kbc_enable_mux_ports;
	}

#endif

	aux_enable = jz_kbc_enable_aux_port;
	if (aux_enable())
		goto err_free_ports;

	return 0;

err_free_ports:
	jz_kbc_free_aux_ports();
	return error;
}

static int __devinit jz_kbc_setup_kbd(void)
{
	int error;

	error = jz_kbc_create_kbd_port();
	if (error)
		return error;

	error = jz_kbc_enable_kbd_port();
	if (error)
		goto err_free_port;

	return 0;

err_free_port:
	jz_kbc_free_kbd_port();
	return error;
}

static int __devinit jz_kbc_probe(struct platform_device *dev)
{
	int error;

	error = jz_kbc_platform_init();
	if (error)
		return error;

	error = jz_kbc_controller_selftest();
	if (error)
		return error;

	error = jz_kbc_controller_init();
	if (error)
		return error;

	if (!jz_kbc_noaux) {
		error = jz_kbc_setup_aux();
		if (error && error != -ENODEV && error != -EBUSY)
			goto out_fail;
	}

	if (!jz_kbc_nokbd) {
		error = jz_kbc_setup_kbd();
		if (error)
			goto err_free_aux;
	}

	error = request_irq(IRQ_KBC, jz_kbc_interrupt, IRQF_SHARED,
			"jz_kbc", jz_kbc_platform_device);
	if (error)
		goto err_free;
	/*
	 * Ok, everything is ready, let's register all serio ports
	 */
	jz_kbc_register_ports();

	return 0;


err_free:
	jz_kbc_free_kbd_port();
err_free_aux:
	jz_kbc_free_aux_ports();	/* in case KBD failed but AUX not */
out_fail:
	jz_kbc_controller_reset();

	return error;
}

static int __devexit jz_kbc_remove(struct platform_device *dev)
{
	jz_kbc_unregister_ports();
	free_irq(IRQ_KBC, jz_kbc_platform_device);
	jz_kbc_controller_reset();

	return 0;
}

static struct platform_driver jz_kbc_driver = {
	.driver		= {
		.name	= "jz-kbc",
		.owner	= THIS_MODULE,
	},
	.probe		= jz_kbc_probe,
	.remove		= __devexit_p(jz_kbc_remove),
	.shutdown	= jz_kbc_shutdown,
#ifdef CONFIG_PM
	.suspend	= jz_kbc_suspend,
	.resume		= jz_kbc_resume,
#endif
};

static int __init jz_kbc_init(void)
{
	int err;

	dbg_init();

	err = platform_driver_register(&jz_kbc_driver);
	if (err)
		return err;

	jz_kbc_platform_device = platform_device_alloc("jz-kbc", -1);
	if (!jz_kbc_platform_device) {
		err = -ENOMEM;
		goto err_unregister_driver;
	}

	err = platform_device_add(jz_kbc_platform_device);
	if (err)
		goto err_free_device;

	panic_blink = jz_kbc_panic_blink;

	return 0;

err_free_device:
	platform_device_put(jz_kbc_platform_device);
err_unregister_driver:
	platform_driver_unregister(&jz_kbc_driver);

	return err;
}

static void __exit jz_kbc_exit(void)
{
	platform_device_unregister(jz_kbc_platform_device);
	platform_driver_unregister(&jz_kbc_driver);

	panic_blink = NULL;
}

module_init(jz_kbc_init);
module_exit(jz_kbc_exit);
