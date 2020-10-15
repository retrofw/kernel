////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) Newport Media Inc.  All rights reserved.
//
// Module Name:  NMICMN.C
//
// Author : K.Yu
//
// Date : 6th June. 2006
//
//////////////////////////////////////////////////////////////////////////////
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/jzsoc.h>

#include "dvb.h"
//#include "../../work/fpga-test/tssi/trunk/drivers/char/nmi_i2c.h"

#define NPM_IOCTL_DVB_RUN_CORE	1
#define NPM_IOCTL_DVB_RUN_LNK	2

static NMIDTVVTBL dtv;
static NMIDTVVTBL *ptv = &dtv;


extern void nmi_bus_read(u32 adr, u8 *b, u32 sz);
extern void nmi_bus_write(u32 adr, u8 *b, u32 sz);

typedef struct {
	NMITV				dtvtype;
	NMIBUSTYPE 	bustype;
	uint32_t			dbg;
	int					crystalindex;
	NMIHLPVTBL 	hlp;
	NMIDTVVTBL 	dtv;
	int					chipver;
} NMICHIP;


static NMICHIP chip;
static NMICHIP *pchp = &chip;

/********************************************
	Debug Functions
********************************************/
static void nmi_debug(uint32_t flag, char *fmt,...)
{
#if 0
	char buf[256];
	va_list args;
	int len;

	if (flag & pchp->dbg) { 
		va_start(args, fmt);
#ifdef _WINXP_KERNEL_
		RtlStringCbVPrintfA(buf, 256, fmt, args);
#else
		len = vsprintk(buf, fmt, args);
#endif
		va_end(args);

		if (pchp->hlp.nmi_log) {
			pchp->hlp.nmi_log(buf);
		}
	}
#endif

	return;
}

/********************************************
	Helper Functions
********************************************/
static void nmi_delay(uint32_t msec)
{
	mdelay(msec);
//	if (pchp->hlp.nmi_delay) {
	//	pchp->hlp.nmi_delay(msec);
//	}
}

static uint32_t nmi_get_tick(void)
{
	uint32_t tick = 0;

	if (pchp->hlp.nmi_get_tick) {
		tick = pchp->hlp.nmi_get_tick();
	}

	return tick;
}

static void nmi_dma_read(void *pv)
{
	if (pchp->hlp.nmi_dma_read) {
		pchp->hlp.nmi_dma_read(pv);
	}
}

static void nmi_tbl_read(void *pv)
{
	if (pchp->hlp.nmi_dvb_read_tbl) {
		pchp->hlp.nmi_dvb_read_tbl(pv);
	}
}

/********************************************
	Bus Read/Write Functions
********************************************/
static uint8_t rReg8(uint32_t adr)
{
	uint8_t val;

	if (pchp->hlp.nmi_read_reg != NULL) {
		pchp->hlp.nmi_read_reg(adr, (uint8_t *)&val, 1);
		return val;
	}

	return 0; 
}

static void wReg8(uint32_t adr, uint8_t val)
{
	if (pchp->hlp.nmi_write_reg != NULL) {
		pchp->hlp.nmi_write_reg(adr, (uint8_t *)&val, 1);
	}

	return; 
}

static uint32_t rReg32(uint32_t adr)
{
	uint32_t val;

	if (pchp->hlp.nmi_read_reg != NULL) {
		pchp->hlp.nmi_read_reg(adr, (uint8_t *)&val, 4);
		return val;
	}

	return 0; 
}

static void wReg32(uint32_t adr, uint32_t val)
{
	if (pchp->hlp.nmi_write_reg != NULL) {
		pchp->hlp.nmi_write_reg(adr, (uint8_t *)&val, 4);
	}
	return; 
}

/********************************************
	DVB Functions
********************************************/
#include "dvb.c"

/********************************************
	NMI Chip Initialize Functions
********************************************/
NMIDTVVTBL *nmi_common_init(NMICMN *pdrv)
{
	NMIDTVVTBL *ptv;

	memset((void *)pchp, 0, sizeof(NMICHIP));

	/* save the driver info */
	pchp->hlp.nmi_write_reg = pdrv->tbl.nmi_write_reg;
	pchp->hlp.nmi_read_reg = pdrv->tbl.nmi_read_reg;
	pchp->hlp.nmi_delay = pdrv->tbl.nmi_delay;
	pchp->hlp.nmi_get_tick = pdrv->tbl.nmi_get_tick;
	pchp->hlp.nmi_log = pdrv->tbl.nmi_log;
	pchp->hlp.nmi_dma_read = pdrv->tbl.nmi_dma_read;
	pchp->hlp.nmi_dvb_read_tbl = pdrv->tbl.nmi_dvb_read_tbl;

	pchp->dtvtype = pdrv->dtvtype;
	pchp->dbg = pdrv->dbgflag;
	pchp->bustype = pdrv->bustype;
	pchp->crystalindex = pdrv->crystalindex;
	pchp->chipver = pdrv->chipver;


	if (pchp->dtvtype == DVB) {
		ptv = dvb_init();
        printk(KERN_INFO "After Init nmi_common_init!!\n");
	} else {
		return ((void *)0);
	}

	return ptv;
}
static int nmi_open(struct inode *inode, struct file *file)
{
        return 0;
}

static int nmi_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	
	switch(cmd) {
		case NPM_IOCTL_DVB_RUN_CORE:
			{
				DVBCORE *dc = (DVBCORE *)arg;

				DVBTUNE tune;
				DVBRUN core;

				//              down(&dd->mu);
				memset((void *)&tune, 0, sizeof(DVBTUNE));
				tune.frequency = dc->frequency;
				tune.bandwidth = dc->bandwidth;
				printk("\r\ntune.frequency:%d tune.bandwidth:%d\r\n",tune.frequency,tune.bandwidth);
				ptv->nmi_config_tuner((void *)&tune);
				memset((void *)&core, 0, sizeof(DVBRUN));
				ptv->nmi_config_demod((void *)&core);

				if (core.lock) {
					printk("RF is Locked success!\r\n");
					dc->initok = 1;
				} else {
					printk(" Locked fail!\r\n");
					dc->initok = 0;
				}
				//              up(&dd->mu);
			}
			break;
		case NPM_IOCTL_DVB_RUN_LNK:
                {
                        int i;
                        DVBLNK *dl = (DVBLNK *)arg;
                        DVBLNKCTL lnk;

//                      down(&dd->mu);
                        memset((void *)&lnk, 0, sizeof(DVBLNKCTL));
                        if (dl->mode == 1) {            /* DVB-T */
                                lnk.mode = 1;
                                lnk.tsmode = 1;  //add by jim gao 2-5-09
                                if (dl->u.t.enfilt) {
                                        lnk.entfilt = 1;
                                        lnk.ntfilt = dl->u.t.npid;
                                        if (lnk.ntfilt > 16)
                                                lnk.ntfilt = 16;
                                        for (i = 0; i < lnk.ntfilt; i++) {
                                                lnk.tfilt[i] = dl->u.t.pid[i];
                                        }
                                }
                                else
                                {
                                   lnk.entfilt=0;
                                }

                                /**
                                   Comment: this is to set the chip's internal buffering size. The interrupt will happen
                                   when chip received TS packets reach the value set in here.  The maximum size is
                                   1742 which means 1742 * 188 TS packets.  Make sure that the bost biffer is large enough
                                   to match the setting here.
                                **/
                                lnk.tsz = 348;

                        } else {                /* DVB-H */

                                /* reset the link layer and counter */
                                ptv->nmi_dvb_rst_lnk();
                                ptv->nmi_dvb_rst_lnk_cnt();

                                lnk.mode = 0;                                           /* DVB-H mode */
                                lnk.tsmode = 0;                                 /* No TS output */
                                lnk.npid = dl->u.h.npid;
                                if (lnk.npid > 8)
                                        lnk.npid = 8;
                                for (i = 0; i < lnk.npid; i++) {
                                        lnk.pid[i] = dl->u.h.pid[i];
                                        lnk.fec[i] = dl->u.h.fec[i];
                                        lnk.type[i] = dl->u.h.type[i];
                                        printk("\r\n[DVB]: pid (%04x), fec (%02x), type (%d)\r\n", lnk.pid[i], lnk.fec[i], lnk.type[i]);
                                }
                        }

                     printk("reset link\r\n");
                        if ( ptv->nmi_dvb_rst_lnk != NULL )
                                ptv->nmi_dvb_rst_lnk();

                        if ( ptv->nmi_dvb_rst_lnk_cnt != NULL )
                                ptv->nmi_dvb_rst_lnk_cnt();

                     printk("config  link(mac layer)\n");
                        ptv->nmi_config_mac((void *)&lnk);
//                      up(&dd->mu);
                }

                break;


	}

	return ret;
}
static struct file_operations nmi_fops = {
        .owner  = THIS_MODULE,
        .open   = nmi_open,
        .ioctl  = nmi_ioctl,
};

static struct miscdevice nmi_dev = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "nmi",
        .fops   = &nmi_fops,
};

#include "i2c.c"
static int __init nmi_dvb_init(void)
{
	int rv;
        NMICMN cmn;

        /**
                init bus
        **/
	
//	__gpio_as_output0(32+27);
//	mdelay(10);
//	__gpio_as_output1(32+27);

        if (nmi_bus_init() < 0) {
                printk( "[NMIDVB]: Init, Failed to init bus...\n");
		return -1;
        }


        /* init nmi chip */
        cmn.dtvtype                                                     = DVB;
        cmn.bustype                                                     = _I2C_;
        cmn.dbgflag                                                     = _ERR_|_INFO_;
        cmn.chipver                                                     = 1;            /* 1: c0
and above, 0: a0 */
        cmn.crystalindex                                        = 5;            /* PLL-26 MHz */
        cmn.tbl.nmi_write_reg                   = nmi_bus_write;
        cmn.tbl.nmi_read_reg                    = nmi_bus_read;
        cmn.tbl.nmi_delay                                       = nmi_delay;
       // cmn.tbl.nmi_get_tick                    = nmi_get_tick;
       // cmn.tbl.nmi_log                                                 = nmi_log;
//        cmn.tbl.nmi_dvb_read_tbl        = nmi_bus_read_tbl;
        ptv = nmi_common_init(&cmn);
	if ((rv = misc_register(&nmi_dev)) < 0) {
                printk("failed to register misc device\n.");
		return -1;
        }
	return 0;
}

static void __exit nmi_dvb_exit(void)
{
	nmi_bus_deinit();
	misc_deregister(&nmi_dev);
	printk("DVB Module removed.\n");
}

module_init(nmi_dvb_init);
module_exit(nmi_dvb_exit);

MODULE_LICENSE("GPL");
