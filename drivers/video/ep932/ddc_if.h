
#ifndef DDC_IF_H
#define DDC_IF_H


#include <linux/string.h>
#include "ep932api.h"
#include "./i2c_drivers/hdmi.h"


extern smbus_status status;
extern unsigned char ddc_data[128];	 // the ddc buffer

// edid status error code
typedef enum {
	// master
	EDID_STATUS_SUCCESS = 0x00,
	EDID_STATUS_PENDING,		// SMBUS_STATUS_Abort,
	EDID_STATUS_NOACT = 0x02,
	EDID_STATUS_TIMEOUT,
	EDID_STATUS_ARBITRATIONLOSS = 0x04,
	EDID_STATUS_EXTENSIONOVERFLOW,
	EDID_STATUS_CHECKSUMERROR
} edid_status;


/* downstream HDCP control interface */

extern unsigned char downstream_rx_read_bksv(unsigned char *pbksv);
extern unsigned char downstream_rx_bcaps(void);
extern void downstream_rx_write_ainfo(char ainfo);
extern void downstream_rx_write_an(unsigned char *pan);
extern void downstream_rx_write_aksv(unsigned char *paksv);
extern unsigned char downstream_rx_read_ri(unsigned char *pri);
extern void downstream_rx_read_bstatus(unsigned char *pbstatus);
extern void downstream_rx_read_sha1_hash(unsigned char *psha);
extern unsigned char downstream_rx_read_ksv_fifo(unsigned char *pbksv, unsigned char index, unsigned char devcount);


/* downstream EDID control interface */

extern unsigned char downstream_rx_poll_edid(void);
extern edid_status downstream_rx_read_edid(unsigned char *pedid);


#endif // DDC_IF_H


