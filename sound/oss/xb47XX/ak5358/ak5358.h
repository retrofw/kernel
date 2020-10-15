#ifndef __AK5358_H
#define __AK5358_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input.h>

#include <linux/ak5358.h>

#ifdef CONFIG_AK5358_DEBUG
static int noinline __attribute__((unused))
ak5358_printk(const char *comp, const char *fmt, ...)
{
	va_list args;
	int rtn;

        printk("===>CPU%d: AK5358(%s): ", smp_processor_id(), comp);
	va_start(args, fmt);
        rtn = vprintk(fmt, args);
	va_end(args);

	return rtn;
}
#endif

#endif
