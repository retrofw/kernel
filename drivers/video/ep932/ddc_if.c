#include <linux/kernel.h>
#include <linux/delay.h>

#include "ddc_if.h"


#define HDCP_RX_ADDR            0x74     // HDCP RX address
#define EDID_ADDR      		0xA0     // EDID address
#define EDID_SEGMENT_PTR	0x60

#define HDCP_RX_BKSV_ADDR       0x00     // HDCP RX, BKSV register address
#define HDCP_RX_RI_ADDR         0x08     // HDCP RX, RI register address
#define HDCP_RX_AKSV_ADDR       0x10     // HDCP RX, AKSV register address
#define HDCP_RX_AINFO_ADDR      0x15     // HDCP RX, AINFO register address
#define HDCP_RX_AN_ADDR         0x18     // HDCP RX, AN register address
#define HDCP_RX_SHA1_HASH_ADDR  0x20     // HDCP RX, SHA-1 Hash Value Start address
#define HDCP_RX_BCAPS_ADDR      0x40     // HDCP RX, BCAPS register address
#define HDCP_RX_BSTATUS_ADDR    0x41     // HDCP RX, BSTATUS register address
#define HDCP_RX_KSV_FIFO_ADDR   0x43     // HDCP RX, KSV FIFO Start address



smbus_status status;

unsigned char ddc_data[128];
unsigned char tempbit;

/* downstream HDCP control */

unsigned char downstream_rx_read_bksv(unsigned char *pbksv)
{
	int i, j;
	status = jz_i2c_hdcp_rx_read(HDCP_RX_BKSV_ADDR, pbksv, 5);
	if(status != SMBUS_STATUS_SUCCESS) {
		DBG_PRINTK(("ERROR: BKSV read - DN DDC %d\r\n", (int)status));
		return 0;
	}

	i = 0;
	j = 0;
	while (i < 5) {
		tempbit = 1;
		while (tempbit) {
			if (pbksv[i] & tempbit) j++;
			tempbit <<= 1;
		}

		i++;
	}
	if(j != 20) {
		DBG_PRINTK(("ERROR: BKSV read - Key Wrong\r\n"));
		DBG_PRINTK(("ERROR: BKSV=0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\r\n", (unsigned int)pbksv[0], (unsigned int)pbksv[1], (unsigned int)pbksv[2], (unsigned int)pbksv[3], (unsigned int)pbksv[4]));

		return 0;
	}

	return 1;
}

unsigned char downstream_rx_bcaps(void)
{
 	jz_i2c_hdcp_rx_read(HDCP_RX_BCAPS_ADDR, ddc_data, 1);
	return ddc_data[0];
}

void downstream_rx_write_ainfo(char ainfo)
{
	jz_i2c_hdcp_rx_write(HDCP_RX_AINFO_ADDR, &ainfo, 1);
}

void downstream_rx_write_an(unsigned char *pan)
{
	jz_i2c_hdcp_rx_write(HDCP_RX_AN_ADDR, pan, 8);
}

void downstream_rx_write_aksv(unsigned char *paksv)
{
	jz_i2c_hdcp_rx_write(HDCP_RX_AKSV_ADDR, paksv, 5);
}

unsigned char downstream_rx_read_ri(unsigned char *pri)
{
	status = jz_i2c_hdcp_rx_read(HDCP_RX_RI_ADDR, pri, 2);
	if(status != SMBUS_STATUS_SUCCESS) {
		DBG_PRINTK(("ERROR: Rx Ri read - MCU IIC %d\r\n", (int)status));

		return 0;
	}

	return 1;
}

void downstream_rx_read_bstatus(unsigned char *pbstatus)
{
	jz_i2c_hdcp_rx_read(HDCP_RX_BSTATUS_ADDR, pbstatus, 2);
}

void downstream_rx_read_sha1_hash(unsigned char *psha)
{
	jz_i2c_hdcp_rx_read(HDCP_RX_SHA1_HASH_ADDR, psha, 20);
}

// retrive a 5 byte KSV at "index" from FIFO
unsigned char downstream_rx_read_ksv_fifo(unsigned char *pbksv, unsigned char index, unsigned char devcount)
{
	int i, j;

	// try not to re-read the previous KSV
	if(index == 0) { 
		status = jz_i2c_hdcp_rx_read(HDCP_RX_KSV_FIFO_ADDR, ddc_data, min(devcount, 25));
	}
	memcpy(pbksv, ddc_data+(index*5), 5);

	if(status != SMBUS_STATUS_SUCCESS) {
		DBG_PRINTK(("ERROR: KSV FIFO read - DN DDC %d\r\n", (int)status));
		return 0;
	}

	i = 0;
	j = 0;
	while (i < 5) {
		tempbit = 1;
		while (tempbit) {
			if (pbksv[i] & tempbit) j++;
			tempbit <<= 1;
		}
		i++;
	}
	if(j != 20) {
		DBG_PRINTK(("ERROR: KSV FIFO read - Key Wrong\r\n"));

		return 0;
	}

	return 1;
}


// downstream edid control
unsigned char downstream_rx_poll_edid(void)
{
	status = jz_i2c_edid_read(0, ddc_data, 1);

	if(status != SMBUS_STATUS_SUCCESS) // can't read EDID
	{
		return 2;
	}
	if(ddc_data[0] != 0x00)				// EDID header fail
	{
		return 2;
	}
	return 0;							// Read EDID success

}

edid_status downstream_rx_read_edid(unsigned char *pedid)
{
	int i;
	unsigned char seg_ptr, blockcount, block1found, chksum;

	
	status = jz_i2c_edid_read(0, pedid, 128);
	for(i=0; i<128; ++i) {
		if(i%16 == 0) DBG_PRINTK(("\r\n"));
		if(i%8 == 0) DBG_PRINTK((" "));
		DBG_PRINTK(("0x%02X, ", (int)pedid[i] ));
	}

	if(status != SMBUS_STATUS_SUCCESS) {
		DBG_PRINTK(("ERROR: EDID b0 read - DN DDC %d\r\n", (int)status));
		return status;
	}
	DBG_PRINTK(("EDID b0 read:"));
	for(i=0; i<128; ++i) {
		if(i%16 == 0) DBG_PRINTK(("\r\n"));
		if(i%8 == 0) DBG_PRINTK((" "));
		DBG_PRINTK(("0x%02x, ", (int)pedid[i] ));
	}
	DBG_PRINTK(("\r\n"));

	if( (pedid[0] != 0x00) ||
	    (pedid[1] != 0xFF) ||
	    (pedid[2] != 0xFF) ||
	    (pedid[3] != 0xFF) ||
	    (pedid[4] != 0xFF) ||
	    (pedid[5] != 0xFF) ||
	    (pedid[5] != 0xFF) ||
	    (pedid[7] != 0x00))
	{
		return EDID_STATUS_NOACT;
	}

	// Check EDID
	if(pedid[126] > 8) {
		DBG_PRINTK(("ERROR: EDID Check failed, pedid[126]=0x%02X > 8\n\r", (int)pedid[126] ));
		return EDID_STATUS_EXTENSIONOVERFLOW;
	}

	// =========================================================
	// II. Read other blocks and find Timing Extension Block

	blockcount = pedid[126];
	block1found = 0;
	for (seg_ptr = 1; seg_ptr <= blockcount; ++seg_ptr) {

		status = jz_i2c_edid_read((seg_ptr & 0x01) << 7, ddc_data, 128);
		if(status != SMBUS_STATUS_SUCCESS) {
			DBG_PRINTK(("ERROR: EDID bi read - DN DDC %d\r\n", (int)status));
			return status;
		}

		if(ddc_data[0] == 0x02 && block1found == 0) {
			block1found = 1;
			memcpy(&pedid[128], ddc_data, 128);
		}

		DBG_PRINTK (("EDID b%d read:", (int)seg_ptr));
		for(i=0; i<128; ++i) {
//			if(i%16 == 0) DBG_PRINTK(("\r\n"));
//			if(i%8 == 0) DBG_PRINTK((" "));
			if(i%16 == 0) printk(("\r\n"));
			if(i%8 == 0)printk((" "));
//			DBG_PRINTK(("0x%02X, ", (int)ddc_data[i] ));
			printk("0x%02X, ", (int)ddc_data[i]);
		}
		DBG_PRINTK(("\r\n"));
	}

	// Check CheckSum
	chksum = 0;
	for(i=0; i<((block1found)?256:128); ++i) {
		chksum += pedid[i];
	}
	if(chksum != 0) {
		return EDID_STATUS_CHECKSUMERROR;
	}
	if(block1found) {
		pedid[126] = 1;
	}
	else {
		pedid[126] = 0;
	}

	return EDID_STATUS_SUCCESS;
}


