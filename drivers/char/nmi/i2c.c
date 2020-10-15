#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp_lock.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <asm/jzsoc.h>
#include "i2c.h"


static struct i2c_client  *pnmicli;


/*
*	I2C read/write routine
*/
void nmi_bus_write(u32 adr, u8 *b, u32 sz)
{
	int ret;
	u8 dat[16];
	struct i2c_msg msg;

    if(!pnmicli) {
        printk("[NMI] i2c write, not init...\n");
        return;
    }

	msg.addr = pnmicli->addr;
	msg.flags = 0;

	switch (sz) {
	case 1:
		dat[0] = 0x30;
		dat[1] = (u8)(adr >> 16);
		dat[2] = (u8)(adr >> 8);
		dat[3] = (u8)adr;
		dat[4] = 0x00;
		dat[5] = 0x01;
		dat[6] = b[0];
		msg.len = 7;
		break;
	case 4:
		dat[0] = 0x90;
		dat[1] = (u8)(adr >> 16);
		dat[2] = (u8)(adr >> 8);
		dat[3] = (u8)adr;
		dat[4] = 0x00;
		dat[5] = 0x04;
		dat[6] = b[0];
		dat[7] = b[1];
		dat[8] = b[2];
		dat[9] = b[3];
		msg.len = 10;
		break;
	default:
		return;
	}
	msg.buf = &dat[0];

	ret = i2c_transfer(pnmicli->adapter, &msg, 1);
	if (ret != 1) {
		printk("[NMI] i2c, failed write, (%d)\n", ret);
	}

	return;	
}

void nmi_bus_read(u32 adr, u8 *b, u32 sz)
{
	int ret;
	u8 dat[6];
	struct i2c_msg msgs[2];
 
    if(!pnmicli) {
		printk("[NMI] i2c read, not init...\n");
		return;
    }

	switch (sz) {
	case 1:
		dat[0] = 0x20;
		dat[1] = (u8)(adr >> 16);
		dat[2] = (u8)(adr >> 8);
		dat[3] = (u8)adr;
		dat[4] = 0x00;
		dat[5] = 0x01;
		break;
	case 4:
		dat[0] = 0x80;
		dat[1] = (u8)(adr >> 16);
		dat[2] = (u8)(adr >> 8);
		dat[3] = (u8)adr;
		dat[4] = 0x00;
		dat[5] = 0x04;
		break;
	default:
		return;
	}

	msgs[0].addr = pnmicli->addr;
	msgs[0].flags = 0;
	msgs[0].len = 6;
	msgs[0].buf = &dat[0];

	msgs[1].addr = pnmicli->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sz;
	msgs[1].buf = b;

	ret = i2c_transfer(pnmicli->adapter, msgs, 2);
	if (ret != 2) {
		printk("[NMI] i2c read, failed to read, (%d)\n", ret);
	}

	return;
}

#if 0
void nmi_bus_read(u32 adr, u8 *b, u32 sz)
{
	int ret;
	u8 dat[6];
	struct i2c_msg msgs[2];
 
    if(!pnmicli) {
		printk("[NMI] i2c read, not init...\n");
		return;
    }

	switch (sz) {
	case 1:
		dat[0] = 0x20;
		dat[1] = (u8)(adr >> 16);
		dat[2] = (u8)(adr >> 8);
		dat[3] = (u8)adr;
		dat[4] = 0x00;
		dat[5] = 0x01;
		break;
	case 4:
		dat[0] = 0x80;
		dat[1] = (u8)(adr >> 16);
		dat[2] = (u8)(adr >> 8);
		dat[3] = (u8)adr;
		dat[4] = 0x00;
		dat[5] = 0x04;
		break;
	default:
		return;
	}

	msgs[0].addr = pnmicli->addr;
	msgs[0].flags = 0;
	msgs[0].len = 6;
	msgs[0].buf = &dat[0];

	msgs[1].addr = pnmicli->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sz;
	msgs[1].buf = b;

	printk("FUNC : %s, LINE: %d, pnmicli->addr = %d\n", __func__, __LINE__, pnmicli->addr);
	ret = i2c_transfer(pnmicli->adapter, msgs, 2);
	if (ret != 2) {
		printk("[NMI] i2c read, failed to read, (%d)\n", ret);
	}

	return;
}
#endif

#if 0
static int nmi_bus_read_tbl(void *pv)
{
	struct nmi_eth_dev *ed = (struct nmi_eth_dev *)this_device->ml_priv;
	struct nmi_dvb_dev *dd = (struct nmi_dvb_dev *)ed->pv;
	DVBTBLREAD *p = (DVBTBLREAD *)pv;
	u32 sadr = p->sadr;
	int sz = p->sz;

	if (sz <= (4 * 1024)) {
		if (dd->bt != NULL && dd->btsz != NULL) { 
			u8 *b = dd->bt;
			nmi_bus_read(sadr, b, sz); 
			*dd->btsz = sz;
		} else {
			printk(_DERR_, "[NMIDVB]: Failed, table pointer invalid...\n");
		}
	} else {
		printk(_DERR_, "[NMIDVB]: Failed, table size exceed the buffer limit, (%d)\n", sz);
	}

	return 0;
}
#endif

#if 1
static int
nmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int err;

        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                err = -ENODEV;
                return err;
        }

        pnmicli = client;
	printk("maxueyue %d\n", pnmicli->addr);
	

        return 0;
}


static int nmi_remove(struct i2c_client *client)
{
        return 0;
}
static const struct i2c_device_id nmi_id[] = {
        { "nmi", 0 },
        { }     /* Terminating entry */
};
MODULE_DEVICE_TABLE(i2c, nmi_id);


static struct i2c_driver nmiI2cDriver = {
        .probe          = nmi_probe,
        .remove         = nmi_remove,
        .id_table       = nmi_id,
        .driver = {
                .name   = "nmi",
        },
};

#endif 

static int  nmi_bus_init(void)
{	
	printk("FUNC :%s, LINE : %d\n", __func__, __LINE__);
    return i2c_add_driver(&nmiI2cDriver);
}

static void nmi_bus_deinit(void)
{
    i2c_del_driver(&nmiI2cDriver);
}
//EXPORT_SYMBOL(nmi_bus_read);
//EXPORT_SYMBOL(nmi_bus_write);

//module_init(nmi_bus_init);
//module_exit(nmi_bus_deinit);

//MODULE_LICENSE("GPL");

