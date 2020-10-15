#ifndef __HDMI_H__
#define __HDMI_H__
	
extern int jz_i2c_ep932_write(unsigned char adress, unsigned char *txData, int length);
extern int jz_i2c_ep932_read(unsigned char adress, unsigned char *txData, int length);

extern int jz_i2c_edid_write(unsigned char adress, unsigned char *txData, int length);
extern int jz_i2c_edid_read(unsigned char adress, unsigned char *txData, int length);

extern int jz_i2c_hdcp_rx_write(unsigned char adress, unsigned char *txData, int length);
extern int jz_i2c_hdcp_rx_read(unsigned char adress, unsigned char *txData, int length);

extern int jz_i2c_hey_write(unsigned char adress, unsigned char *txData, int length);
extern int jz_i2c_hey_read(unsigned char adress, unsigned char *txData, int length);

#endif
