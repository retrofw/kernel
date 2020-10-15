/*
 * linux/arch/mips/jz4760/i2c.c
 * 
 * Jz4760 I2C routines.
 * 
 * Copyright (C) 2005,2006 Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
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
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/addrspace.h>

#include <asm/jzsoc.h>

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0

#define TIMEOUT         1000

/*
 * I2C bus protocol basic routines
 */
static int i2c_put_data(unsigned char data)
{
	unsigned int timeout = TIMEOUT*10;

	__i2c_write(data);
	__i2c_set_drf();
	while (__i2c_check_drf() != 0);
	while (!__i2c_transmit_ended());
	while (!__i2c_received_ack() && timeout)
		timeout--;

	if (timeout)
		return 0;
	else
		return -ETIMEDOUT;
}

#ifdef CONFIG_JZ_TPANEL_ATA2508
static int i2c_put_data_nack(unsigned char data)
{
	unsigned int timeout = TIMEOUT*10;

	__i2c_write(data);
	__i2c_set_drf();
	while (__i2c_check_drf() != 0);
	while (!__i2c_transmit_ended());
	while (timeout--);
		return 0;
}
#endif

static int i2c_get_data(unsigned char *data, int ack)
{
	int timeout = TIMEOUT*10;

	if (!ack)
		__i2c_send_nack();
	else
		__i2c_send_ack();

	while (__i2c_check_drf() == 0 && timeout)
		timeout--;

	if (timeout) {
		if (!ack)
			__i2c_send_stop();
		*data = __i2c_read();
		__i2c_clear_drf();
		return 0;
	} else
		return -ETIMEDOUT;
}

/*
 * I2C interface
 */
void i2c_open(void)
{
	__i2c_set_clk(jz_clocks.extalclk, 10000); /* default 10 KHz */
	__i2c_enable();
}

void i2c_close(void)
{
	udelay(300); /* wait for STOP goes over. */
	__i2c_disable();
}

void i2c_setclk(unsigned int i2cclk)
{
	__i2c_set_clk(jz_clocks.extalclk, i2cclk);
}

int i2c_lseek(unsigned char device, unsigned char offset)
{
	__i2c_send_nack();	/* Master does not send ACK, slave sends it */
	__i2c_send_start();
	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0)
		goto device_err;
	if (i2c_put_data(offset) < 0)
		goto address_err;
	return 0;
 device_err:
	printk(KERN_DEBUG "No I2C device (0x%02x) installed.\n", device);
	__i2c_send_stop();
	return -ENODEV;
 address_err:
	printk(KERN_DEBUG "No I2C device (0x%02x) response.\n", device);
	__i2c_send_stop();
	return -EREMOTEIO;
}

int i2c_read(unsigned char device, unsigned char *buf,
	       unsigned char address, int count)
{
	int cnt = count;
	int timeout = 5;

L_try_again:

	if (timeout < 0)
		goto L_timeout;

	__i2c_send_nack();	/* Master does not send ACK, slave sends it */
	__i2c_send_start();
	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0)
		goto device_werr;
	if (i2c_put_data(address) < 0)
		goto address_err;

	__i2c_send_start();
	if (i2c_put_data( (device << 1) | I2C_READ ) < 0)
		goto device_rerr;
	__i2c_send_ack();	/* Master sends ACK for continue reading */
	while (cnt) {
		if (cnt == 1) {
			if (i2c_get_data(buf, 0) < 0)
				break;
		} else {
			if (i2c_get_data(buf, 1) < 0)
				break;
		}
		cnt--;
		buf++;
	}

	__i2c_send_stop();
	return count - cnt;
 device_rerr:
 device_werr:
 address_err:
	timeout --;
	__i2c_send_stop();
	goto L_try_again;

L_timeout:
	__i2c_send_stop();
	printk("Read I2C device 0x%2x failed.\n", device);
	return -ENODEV;
}

int i2c_write(unsigned char device, unsigned char *buf,
		unsigned char address, int count)
{
	int cnt = count;
	int cnt_in_pg;
	int timeout = 5;
	unsigned char *tmpbuf;
	unsigned char tmpaddr;

	__i2c_send_nack();	/* Master does not send ACK, slave sends it */

 W_try_again:
	if (timeout < 0)
		goto W_timeout;

	cnt = count;
	tmpbuf = (unsigned char *)buf;
	tmpaddr = address;

 start_write_page:
	cnt_in_pg = 0;
	__i2c_send_start();
	if (i2c_put_data( (device << 1) | I2C_WRITE ) < 0)
		goto device_err;
#ifdef CONFIG_JZ_TPANEL_ATA2508
	if (address == 0xff) {
		if (i2c_put_data_nack(tmpaddr) < 0)
			goto address_err;
		while (cnt) {
			if (++cnt_in_pg > 8) {
				__i2c_send_stop();
				mdelay(1);
				tmpaddr += 8;
				goto start_write_page;
			}
			if (i2c_put_data_nack(*tmpbuf) < 0)
				break;
			cnt--;
			tmpbuf++;
		}
	}
	else {

		if (i2c_put_data(tmpaddr) < 0)
			goto address_err;
		while (cnt) {
			if (++cnt_in_pg > 8) {
				__i2c_send_stop();
				mdelay(1);
				tmpaddr += 8;
				goto start_write_page;
			}
			if (i2c_put_data(*tmpbuf) < 0)
				break;
			cnt--;
			tmpbuf++;
		}
	}
#else
	if (i2c_put_data(tmpaddr) < 0)
		goto address_err;
	while (cnt) {
		if (++cnt_in_pg > 8) {
			__i2c_send_stop();
			mdelay(1);
			tmpaddr += 8;
			goto start_write_page;
		}
		if (i2c_put_data(*tmpbuf) < 0)
			break;
		cnt--;
		tmpbuf++;
	}
#endif
	__i2c_send_stop();
	return count - cnt;
 device_err:
 address_err:
	timeout--;
	__i2c_send_stop();
	goto W_try_again;

 W_timeout:
	printk(KERN_DEBUG "Write I2C device 0x%2x failed.\n", device);
	__i2c_send_stop();
	return -ENODEV;
}

EXPORT_SYMBOL(i2c_open);
EXPORT_SYMBOL(i2c_close);
EXPORT_SYMBOL(i2c_setclk);
EXPORT_SYMBOL(i2c_read);
EXPORT_SYMBOL(i2c_write);
