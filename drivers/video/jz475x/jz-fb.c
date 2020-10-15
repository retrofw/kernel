/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Copyright (C) 2005-2010 Ingenic Semiconductor Inc.
 * Author: River Wang <zwang@ingenic.cn>
 *
 *  Based on original version: 
 *  Author: Wolfgang Wang, Lemon Liu, Emily Feng.
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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "jz-fb.h"

#define DRV_NAME "jz-fb"
#define DRV_VERSION "V0.1"

static int default_output_id = 0;

static struct jz_fb_ctrl jz_fb_ctrl;

/* ----------------- COMMON Routines ---------------------- */
#include "common.c"
/* -------------------------------------------------------- */

/* ----------------- OUTPUT Config ------------------------ */
#include "output/config.c"
/* -------------------------------------------------------- */

/* ------------------ Core Function Block. ---------------- */
#include "core/desc.c"
#include "core/hw.c"
#include "core/framebuffer.c"
#include "core/win.c"
#include "core/ot.c"
/* -------------------------------------------------------- */

/* ------------------ OUTPUT ------------------------------ */
#include "output/output.c"
/* -------------------------------------------------------- */

/* ---------------- Driver Interface. --------------------- */
#include "drv/fb-drv.c"
#include "drv/ctrl-drv.c"
/* -------------------------------------------------------- */

static struct jz_fb_ot_info *id_to_ot(int id)
{
	int i;

	for (i = 0; i < sizeof(jz_fb_ots) / sizeof(struct jz_fb_ot_info *); i++) {
		if (jz_fb_ots[i]->id == id)
			return jz_fb_ots[i];
	}

	return NULL;
}

static int jz_fb_drv_alloc(struct jz_fb_ctrl *ctrl,
				struct jz_fb_ot_scan_info *scan)
{
	struct jz_fb_win_info *win;

	int i;

	int rv = 0;

	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		win = alloc_win(scan->max_w, 
			scan->max_h, scan->max_bpp);

		if (!win) {
			E("Failed to allocate win info for FG %d.", i);
			rv = -ENOMEM;

			break;
		}

		win->index = i;
		win->ctrl = ctrl;

		ctrl->win[i] = win;
	}
	
	D("Called.");

	if (rv) {
		while (--i >= 0) {
			if (ctrl->win[i])
				free_win(ctrl->win[i]);
		}

		return rv;
	}

	return 0;
}

static int jz_fb_drv_free(struct jz_fb_ctrl *ctrl)
{
	int i;

	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		if (ctrl->win[i])
			free_win(ctrl->win[i]);
	}

	return 0;
}

static int jz_fb_drv_setup(struct jz_fb_ctrl *ctrl)
{
	int rv;

	int i;

	/* Setup Linux Framebuffer Driver for every window. */
	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		if (ctrl->win) {
			rv = fb_drv_setup(ctrl->win[i]);
			if (rv) {
				return rv;
			}
		}
	}

	/* Setup Control Driver for every output path. */	
	for (i = 0; i < sizeof(jz_fb_ots) / sizeof(struct jz_fb_ot_info *); i++) {
		rv = ctrl_drv_setup(jz_fb_ots[i]);
		if (rv) {
			return rv;
		}
	}

	return 0;
}

static int jz_fb_drv_cleanup(struct jz_fb_ctrl *ctrl)
{
	int rv;

	int i;

	for (i = 0; i < JZ_FB_NR_MAX_FG; i++) {
		if (ctrl->win) {
			rv = fb_drv_cleanup(ctrl->win[i]);
			if (rv) {
				return rv;
			}
		}
	}

	for (i = 0; i < sizeof(jz_fb_ots) / sizeof(struct jz_fb_ot_info *); i++) {
		rv = ctrl_drv_cleanup(jz_fb_ots[i]);
		if (rv) {
			return rv;
		}
	}


	return 0;
}

static int __devinit jz_fb_probe(struct platform_device *dev)
{
	struct jz_fb_ctrl *ctrl = &jz_fb_ctrl;
	struct jz_fb_ot_scan_info scan;

	unsigned long status = 0;

	int rv = 0;

	/* 1. Scan & Initialize All Outputs. */
	jz_fb_ot_scan(ctrl, &scan);

	D("Final: Max: w: %u, h: %u, bpp: %u.", scan.max_w, scan.max_h, scan.max_bpp);
	
	/* 2. Locate the default OT. */
	ctrl->ot = id_to_ot(default_output_id);
	if (!ctrl->ot) {
		E("Default Output not found: ID: %d.\n", default_output_id);
		return rv;
	}
	
	/* 3. All allocations. */
	rv = jz_fb_drv_alloc(ctrl, &scan);
	if (rv) {
		E("Failed to allocate FB Driver Layer.");
		return rv;
	}
	
	set_bit(0, &status);

	/* 4. Select OT & Init HW & FB_INFO. */	
	jz_fb_ot_set_current(ctrl->ot);
	
	if (request_irq(IRQ_LCD, jz_fb_interrupt_handler, IRQF_DISABLED,
				DRV_NAME, 0)) {
		E("Failed to request LCDC IRQ.");
		rv = -EBUSY;
		goto err;
	}
	
	set_bit(1, &status);

	/* 5. DRV Layer Setup. */
	rv = jz_fb_drv_setup(ctrl);
	if (rv) {
		E("Failed to setup DRV layer.");
		goto err;
	}
	
	I("Registered.");

	return rv;

err:
	if (test_bit(0, &status)) {
		jz_fb_drv_free(ctrl);
	}
	
	if (test_bit(1, &status)) {
		jz_fb_hw_ot_disable(ctrl, 1);
		jz_fb_hw_ctrl_disable(ctrl, 1);
		jz_fb_hw_ot_power_off(ctrl, 1);

		free_irq(IRQ_LCD, DRV_NAME);
	}

	return rv;
}

static int __devexit jz_fb_remove(struct platform_device *pdev)
{
	struct jz_fb_ctrl *ctrl= &jz_fb_ctrl;

	jz_fb_hw_ot_disable(ctrl, 1);
	jz_fb_hw_ctrl_disable(ctrl, 1);
	jz_fb_hw_ot_power_off(ctrl, 1);

	free_irq(IRQ_LCD, DRV_NAME);

	jz_fb_drv_cleanup(ctrl);
	jz_fb_drv_free(ctrl);

	return 0;
}

static struct platform_driver jz_fb_driver = {
	.probe	= jz_fb_probe,
	.remove = jz_fb_remove,
	.suspend = jz_fb_suspend,
	.resume = jz_fb_resume,
	.driver = {
		.name = "jz-lcd",
		.owner = THIS_MODULE,
	},
};

static int __init jz_fb_init(void)
{
	return platform_driver_register(&jz_fb_driver);
}

static void __exit jz_fb_cleanup(void)
{
	platform_driver_unregister(&jz_fb_driver);
}

module_init(jz_fb_init);
module_exit(jz_fb_cleanup);
