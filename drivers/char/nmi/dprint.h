#ifndef _DPRINT_H_
#define _DPRINT_H_

#define _DINIT_			0x1
#define _DERR_			0x2
#define _DFUNC_		0x4
#define _DINTR_  		0x8
#define _DIO_			0x10

extern u32 dflag;

#define DPrint(fl, fmt, msg...) \
    if(((fl) & dflag) == (fl)) { printk("%s: ", __FUNCTION__); printk(fmt, ## msg); }

#define DPrint1(fl, fmt, msg...) \
    if(((fl) & dflag) == (fl)) { printk(fmt, ## msg); }

#endif

