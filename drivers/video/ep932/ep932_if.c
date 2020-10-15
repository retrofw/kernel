#include <linux/kthread.h>
#include <linux/delay.h>

#include "ep932_if.h"
#include "ddc_if.h"
#include "ep932settingsdata.h"

#define EP932_ADDR			0x70
#define EP932_ADDR_2			0x72
#define HEY_ADDR			0xA8


extern struct task_struct *hdmi_kthread;
extern int hdmi_init;

// Private data
unsigned char iic_ep932_addr,iic_key_addr;

unsigned short tempushort;

unsigned char temp_data[15];
unsigned char w_data[2];

// Global date for HDMI Transmiter
unsigned char is_hdcp_avmute;
unsigned char is_amute;
unsigned char is_vmute;
unsigned char is_hdmi;
unsigned char is_rsen;
unsigned char cache_ep932_de_control;

// Private Functions
smbus_status iic_write(unsigned char iic_addr, unsigned char byteaddr, unsigned char *data, unsigned int size);
smbus_status iic_read(unsigned char iic_addr, unsigned char byteaddr, unsigned char *data, unsigned int size);


void ep932_if_initial(void)	//customer setting
{
	//EP932_handle = E_handle;
	//KEY_handle = K_handle;
	iic_ep932_addr = 0x70;
	iic_key_addr = 0xA8;

#if 0
// for test only
	DBG_PRINTK(("IIC test write [0x63]0xAA, [0x64]0xA5, [0x65]0x5A\r\n"));

//	while (i++ < 10)
while(1) {
		temp_data[0] = 0xAA;
	printk("write data 0xAA.\n");
	ep932_reg_write(0x10, temp_data, 1);
	mdelay(200);
	memset(temp_data, 0, 128);
	ep932_reg_read(0x10, temp_data, 1);
//	while (i--)
	printk("maxueyue ======================= read data = %#x\n", temp_data[0]);
	mdelay(5000);
}

	ep932_reg_write(0x63, temp_data, 1);
	temp_data[0] = 0xA5;
	ep932_reg_write(0x64, temp_data, 1);
	temp_data[0] = 0x5A;
	ep932_reg_write(0x65, temp_data, 1);

	temp_data[0] = 0x55;
	ep932_reg_write(0x60, temp_data, 1);
	temp_data[0] = 0x99;
	ep932_reg_write(0x61, temp_data, 1);
	temp_data[0] = 0x88;
	ep932_reg_write(0x62, temp_data, 1);

// read for verify

//	ep932_reg_read(EP932_cts, temp_data, 1);
	i2c_read(0x38, temp_data, EP932.cts, 1);
//	DBG_PRINTK(("EP932_cts_0(Reg addr 0x60) = 0x%02X\r\n",(int)temp_data[0]));
	printk("EP932_cts_0(Reg addr 0x60) = 0x%02X\r\n",(int)temp_data[0]);
	i2c_read(0x38, temp_data, 0x61, 1);
//	ep932_reg_read(0x61, temp_data, 1);
//	DBG_PRINTK(("EP932_cts_1(Reg addr 0x61) = 0x%02X\r\n",(int)temp_data[0]));
	printk("EP932_cts_1(Reg addr 0x61) = 0x%02X\r\n",(int)temp_data[0]);
	i2c_read(0x38, temp_data, 0x62, 1);
//	ep932_reg_read(0x62, temp_data, 1);
//	DBG_PRINTK(("EP932_cts_2(Reg addr 0x62) = 0x%02X\r\n",(int)temp_data[0]));
	printk("EP932_cts_2(Reg addr 0x62) = 0x%02X\r\n",(int)temp_data[0]);

	ep932_reg_read(EP932.n, temp_data, 1);
//	DBG_PRINTK(("EP932.n_0(Reg addr 0x63) = 0x%02X\r\n",(int)temp_data[0]));
	printk("EP932.n_0(Reg addr 0x63) = 0x%02X\r\n",(int)temp_data[0]);
	ep932_reg_read(0x64, temp_data, 1);
//	DBG_PRINTK(("EP932.n_1(Reg addr 0x64) = 0x%02X\r\n",(int)temp_data[0]));
printk("EP932.n_1(Reg addr 0x64) = 0x%02X\r\n",(int)temp_data[0]);
	ep932_reg_read(0x65, temp_data, 1);
//	DBG_PRINTK(("EP932.n_2(Reg addr 0x65) = 0x%02X\r\n",(int)temp_data[0]));
	printk("EP932.n_2(Reg addr 0x65) = 0x%02X\r\n",(int)temp_data[0]);

#endif
	
}

void ep932_if_reset(void)
{
	int i;

	// Global date for HDMI Transmiter
	is_hdcp_avmute = 0;
	is_amute = 1;
	is_vmute = 1;
	is_hdmi = 0;
	is_rsen = 0;
	cache_ep932_de_control = 0x03;

	// Initial Settings
	ep932_reg_set_bit(EP932_GENERAL_CONTROL_1, EP932_GENERAL_CONTROL_1__VTX);
	ep932_reg_set_bit(EP932_GENERAL_CONTROL_1, EP932_GENERAL_CONTROL_1__INT_OD);

	// Default Audio Mute
	ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__AMUTE);
	ep932_reg_set_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__CTS_M);
	// Default Video Mute
	ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__VMUTE);

	//
	// Set Default AVI Info Frame
	//
	memset(temp_data, 0x00, 14);

	// Set AVI Info Frame to RGB
	temp_data[1] &= 0x60;
	temp_data[1] |= 0x00; // RGB

	// Set AVI Info Frame to 601
	temp_data[2] &= 0xC0;
	temp_data[2] |= 0x40;

	// Write AVI Info Frame
	temp_data[0] = 0;
	for(i=1; i<14; ++i) {
		temp_data[0] += temp_data[i];
	}
	temp_data[0] = ~(temp_data[0] - 1);
	ep932_reg_write(EP932_AVI_PACKET, temp_data, 14);


	//
	// Set Default ADO Info Frame
	//
	memset(temp_data, 0x00, 6);

	// Write ADO Info Frame
	temp_data[0] = 0;
	for(i=1; i<6; ++i) {
		temp_data[0] += temp_data[i];
	}
	temp_data[0] = ~(temp_data[0] - 1);
	ep932_reg_write(EP932_ADO_PACKET, temp_data, 6);
}

//--------------------------------------------------------------------------------------------------
//
// HDMI Transmiter (EP932-Tx Implementation)
//

void hdmi_tx_power_down(void)
{
	// Software power down
	ep932_reg_clear_bit(EP932_GENERAL_CONTROL_1, EP932_GENERAL_CONTROL_1__PU);
}

void hdmi_tx_power_up(void)
{
	// Software power up
	ep932_reg_set_bit(EP932_GENERAL_CONTROL_1, EP932_GENERAL_CONTROL_1__PU);
}

unsigned char hdmi_tx_htplg(void)
{
	// Software HotPlug Detect
//	return 1;
	ep932_reg_read(EP932_GENERAL_CONTROL_2, temp_data, 1);
	is_rsen = (temp_data[0] & EP932_GENERAL_CONTROL_2__RSEN)? 1:0;
	if(temp_data[0] & EP932_GENERAL_CONTROL_2__HTPLG)
	{
		return 1;
	}
	else
	{
		//DBG_PRINTK(("hdmi_tx_htplg disconnect\r\n"));
		return 0;
	}
	// This is for old DVI monitor compatibility. For HDMI TV, there is no need to poll the EDID.
	return downstream_rx_poll_edid();
}

unsigned char hdmi_tx_rsen(void)
{
	return is_rsen;
}

void hdmi_tx_hdmi(void)
{
	if(!is_hdmi) {
		is_hdmi = 1;
		ep932_reg_set_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__HDMI);
		DBG_PRINTK(("Set to HDMI mode\r\n"));
	}
	if((is_vmute == 0)&&(is_amute == 0)&&(is_hdmi == 1))
		hdmi_init = 0;
}

void hdmi_tx_dvi(void)
{
	if(is_hdmi) {
		is_hdmi = 0;
		ep932_reg_clear_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__HDMI);
		DBG_PRINTK(("Set to DVI mode\r\n"));
	}
}

//------------------------------------
// HDCP

void hdmi_tx_mute_enable(void)
{
	is_hdcp_avmute = 1;
	ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__AMUTE | EP932_COLOR_SPACE_CONTROL__VMUTE);
	ep932_reg_set_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__CTS_M);
}

void hdmi_tx_mute_disable(void)
{
	is_hdcp_avmute = 0;

	if(!is_amute) {
		ep932_reg_clear_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__CTS_M);
		ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__AMUTE);
	}
	if(!is_vmute) {
		ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__VMUTE);
	}
}

void hdmi_tx_hdcp_enable(void)
{
	ep932_reg_set_bit(EP932_GENERAL_CONTROL_5, EP932_GENERAL_CONTROL_5__ENC_EN);
}

void hdmi_tx_hdcp_disable(void)
{
	ep932_reg_clear_bit(EP932_GENERAL_CONTROL_5, EP932_GENERAL_CONTROL_5__ENC_EN);
}

void hdmi_tx_rptr_set(void)
{
	ep932_reg_set_bit(EP932_GENERAL_CONTROL_5, EP932_GENERAL_CONTROL_5__RPTR);
}

void hdmi_tx_rptr_clear(void)
{
	ep932_reg_clear_bit(EP932_GENERAL_CONTROL_5, EP932_GENERAL_CONTROL_5__RPTR);
}

void hdmi_tx_write_an(unsigned char *pan)
{
	ep932_reg_write(EP932_AN, pan, 8);
}

unsigned char hdmi_tx_aksv_rdy(void)
{
	status = ep932_reg_read(EP932_GENERAL_CONTROL_5, temp_data, 1);
	if(status != SMBUS_STATUS_SUCCESS) {
		DBG_PRINTK(("ERROR: AKSV RDY - MCU IIC %d\r\n", (int)status));
		return 0;
	}
	return (temp_data[0] & EP932_GENERAL_CONTROL_5__AKSV_RDY)? 1:0;
}

unsigned char hdmi_tx_read_aksv(unsigned char *paksv)
{
	int i, j;
	unsigned char tmp[1];

	status = ep932_reg_read(EP932_GENERAL_CONTROL_5, tmp, 1);

	status = ep932_reg_read(EP932_AKSV, paksv, 5);
	if(status != SMBUS_STATUS_SUCCESS) {
		DBG_PRINTK(("ERROR: AKSV read - MCU IIC %d\r\n", (int)status));
		return 0;
	}

printk("/************************************************************************************/\n");
	for (i = 0; i < 5; i++)
		printk("paksv[%d] = %#x\n", i, paksv[i]);
printk("/************************************************************************************/\n");
	i = 0;
	j = 0;
	while (i < 5) {
		temp_data[0] = 1;
		while (temp_data[0]) {
			if (paksv[i] & temp_data[0]) j++;
			temp_data[0] <<= 1;
		}
		i++;
	}
	if(j != 20) {
		DBG_PRINTK(("ERROR: AKSV read - key Wrong\r\n"));
		return 0;
	}
	return 1;
}

void hdmi_tx_write_bksv(unsigned char *pbksv)
{
	ep932_reg_write(EP932_BKSV, pbksv, 5);
}

unsigned char hdmi_tx_ri_rdy(void)
{
	ep932_reg_read(EP932_GENERAL_CONTROL_5, temp_data, 1);
	printk("ri tmpdata = %#x\n", temp_data[0]);
	return (temp_data[0] & EP932_GENERAL_CONTROL_5__RI_RDY)? 1:0;
}

unsigned char hdmi_tx_read_ri(unsigned char *pri)
{
	status = ep932_reg_read(EP932_RI, pri, 2);
	if(status != SMBUS_STATUS_SUCCESS) {
		DBG_PRINTK(("ERROR: Tx Ri read - MCU IIC %d\r\n", (int)status));
		return 0;
	}
	return 1;
}

void hdmi_tx_read_m0(unsigned char *pm0)
{
	status = ep932_reg_read(EP932_M0, pm0, 8);
}

smbus_status hdmi_tx_get_key(unsigned char *key)
{
//	return iic_read(HEY_ADDR, 0, key, 512);
	return jz_i2c_hey_read(0, key, 512);
}

//------------------------------------
// Special for config

void hdmi_tx_amute_enable(void)
{
	unsigned char temp_byte[2];
	if(!is_amute) {
		is_amute = 1;
		ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__AMUTE);
		ep932_reg_set_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__CTS_M);

		DBG_PRINTK(("<<< AMute_enable >>>\r\n"));

		//read for verify
		ep932_reg_read(EP932_COLOR_SPACE_CONTROL, temp_byte, 1);
		DBG_PRINTK(("EP932_COLOR_SPACE_CONTROL = 0x%02X\r\n",(int)temp_byte[0]));
		ep932_reg_read(EP932_PIXEL_REPETITION_CONTROL, temp_byte, 1);
		DBG_PRINTK(("EP932_PIXEL_REPETITION_CONTROL = 0x%02X\r\n",(int)temp_byte[0]));
// add end
	}
}

void hdmi_tx_amute_disable(void)
{
	unsigned char temp_byte[2];
	if(is_amute) {
		is_amute = 0;
		if(!is_hdcp_avmute) {
			ep932_reg_clear_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__CTS_M);
			ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__AMUTE);

			DBG_PRINTK(("<<< AMute_disable >>>\r\n"));

			//read for verify
			ep932_reg_read(EP932_COLOR_SPACE_CONTROL, temp_byte, 1);
			DBG_PRINTK(("EP932_COLOR_SPACE_CONTROL = 0x%02X\r\n",(int)temp_byte[0]));
			ep932_reg_read(EP932_PIXEL_REPETITION_CONTROL, temp_byte, 1);
			DBG_PRINTK(("EP932_PIXEL_REPETITION_CONTROL = 0x%02X\r\n",(int)temp_byte[0]));
// add end
		}
	}
	if((is_vmute == 0)&&(is_amute == 0)&&(is_hdmi == 1))
		hdmi_init = 0;
}

void hdmi_tx_vmute_enable(void)
{
	if(!is_vmute) {
		is_vmute = 1;
		ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__VMUTE);

		DBG_PRINTK(("<<< VMute_enable >>>\r\n"));
	}
}

void hdmi_tx_vmute_disable(void)
{
	if(is_vmute) {
		is_vmute = 0;
		if(!is_hdcp_avmute) {
			ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__VMUTE);

			DBG_PRINTK(("<<< VMute_disable >>>\r\n"));
		}
	}
	if((is_vmute == 0)&&(is_amute == 0)&&(is_hdmi == 1))
	hdmi_init = 0;
	//kthread_stop(hdmi_kthread);
}

void hdmi_tx_video_config(pvdo_params params)
{
	int i;
	DBG_PRINTK(("\r\nStart Tx Video Config\r\n"));

	//
	// Disable Video
	//
	ep932_reg_clear_bit(EP932_IIS_CONTROL, EP932_IIS_CONTROL__AVI_EN);

	//
	// Video Settings
	//
	// Interface
	ep932_reg_read(EP932_GENERAL_CONTROL_3, temp_data, 1);
	temp_data[0] &= ~0xF0;
	temp_data[0] |= params->interface & 0xF0;
	ep932_reg_write(EP932_GENERAL_CONTROL_3, temp_data, 1);

	ep932_reg_read(EP932_GENERAL_CONTROL_1, temp_data, 1);
	temp_data[0] &= ~0x0E;
	temp_data[0] |= params->interface & 0x0E;
	ep932_reg_write(EP932_GENERAL_CONTROL_1, temp_data, 1);

	if(params->interface & 0x01) {
		ep932_reg_set_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__FMT12);
	}
	else {
		ep932_reg_clear_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__FMT12);
	}

	// Sync Mode
	switch(params->syncmode) {
		default:
	 	case SYNCMODE_HVDE:
			// Disable E_S.nC
			ep932_reg_clear_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__E_SYNC);
			// Disable DE_G.n
			cache_ep932_de_control &= ~EP932_DE_CONTROL__DE_GEN;
			//ep932_reg_write(EP932_DE_CONTROL, &cache_ep932_de_control, 1);

			// Regular VSO_POL, HSO_POL
			if((params->hvpol & VNEGHPOS) != (ep932_vdo_settings[params->videosettingindex].hvres_type.hvpol & VNEGHPOS)) { // V
				cache_ep932_de_control |= EP932_DE_CONTROL__VSO_POL; // Invert
			}
//	status = ep932_reg_read(EP932_AKSV, pAKSV, 5);
			else {
				cache_ep932_de_control &= ~EP932_DE_CONTROL__VSO_POL;
			}
			if((params->hvpol & VPOSHNEG) != (ep932_vdo_settings[params->videosettingindex].hvres_type.hvpol & VPOSHNEG)) { // H
				cache_ep932_de_control |= EP932_DE_CONTROL__HSO_POL; // Invert
			}
			else {
				cache_ep932_de_control &= ~EP932_DE_CONTROL__HSO_POL;
			}
			DBG_PRINTK(("Set Sync mode to DE mode\r\n"));
			break;

		case SYNCMODE_HV:
			// Disable E_S.nC
			ep932_reg_clear_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__E_SYNC);
			// Enable DE_G.n
			cache_ep932_de_control |= EP932_DE_CONTROL__DE_GEN;
			//ep932_reg_write(EP932_DE_CONTROL, &cache_ep932_de_control, 1);

			// Regular VSO_POL, HSO_POL
			if((params->hvpol & VNEGHPOS) != (ep932_vdo_settings[params->videosettingindex].hvres_type.hvpol & VNEGHPOS)) { // V
				cache_ep932_de_control |= EP932_DE_CONTROL__VSO_POL; // Invert
			}
			else {
				cache_ep932_de_control &= ~EP932_DE_CONTROL__VSO_POL;
			}
			if((params->hvpol & VPOSHNEG) != (ep932_vdo_settings[params->videosettingindex].hvres_type.hvpol & VPOSHNEG)) { // H
				cache_ep932_de_control |= EP932_DE_CONTROL__HSO_POL; // Invert
			}
			else {
				cache_ep932_de_control &= ~EP932_DE_CONTROL__HSO_POL;
			}

			// Set DE generation params
			if(params->videosettingindex < ep932_vdo_settings_max) {
				cache_ep932_de_control &= ~0x03;
				cache_ep932_de_control |= ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].de_gen.de_dly)[0];

				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].de_gen.de_dly)[1];
				ep932_reg_write(EP932_DE_DLY, temp_data, 1);

				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].de_gen.de_top)[0];
				ep932_reg_write(EP932_DE_TOP, temp_data, 1);

				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].de_gen.de_cnt)[1];
				temp_data[1] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].de_gen.de_cnt)[0];
				ep932_reg_write(EP932_DE_CNT, temp_data, 2);

				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].de_gen.de_lin)[1];
				temp_data[1] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].de_gen.de_lin)[0];
				ep932_reg_write(EP932_DE_LIN, temp_data, 2);

				DBG_PRINTK(("Update DE_G.n params %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].de_gen.de_dly));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].de_gen.de_cnt));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].de_gen.de_top));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].de_gen.de_lin));
				DBG_PRINTK(("\r\n"));
			}
			else {
				DBG_PRINTK(("ERROR:.videocode overflow DE_G.n table\r\n"));
			}
			break;

		case SYNCMODE_EMBEDED:
			// Disable DE_G.n
			cache_ep932_de_control &= ~EP932_DE_CONTROL__DE_GEN;
			//ep932_reg_write(EP932_DE_CONTROL, &cache_ep932_de_control, 1);
			// Enable E_S.nC
			ep932_reg_set_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__E_SYNC);

			// Set E_S.nC params
			if(params->videosettingindex < ep932_vdo_settings_max) {

				temp_data[0] = ep932_vdo_settings[params->videosettingindex].e_sync.ctl;
				ep932_reg_write(EP932_EMBEDDED_SYNC, temp_data, 1);
				//DBG_PRINTK(("[0x80]= 0x%02X\r\n",(int)temp_data[0]));


				tempushort = ep932_vdo_settings[params->videosettingindex].e_sync.h_dly;
				if(!(params->interface & 0x04)) { // Mux Mode
					tempushort += 2;
				}

				/*	 // for Big Endean
				temp_data[0] = ((BYTE *)&tempushort)[1];
				temp_data[1] = ((BYTE *)&tempushort)[0];
				ep932_reg_write(EP932_H_DELAY, temp_data, 2);
				*/
				temp_data[0] = ((unsigned char *)&tempushort)[0];		// for Little Endean
				ep932_reg_write(EP932_H_DELAY, temp_data, 1);
				//DBG_PRINTK(("[0x81]= 0x%02X\r\n",(int)temp_data[0]));
				temp_data[0] = ((unsigned char *)&tempushort)[1];   	// for Little Endean
				ep932_reg_write(0x82, temp_data, 1);
				//DBG_PRINTK(("[0x82]= 0x%02X\r\n",(int)temp_data[0]));

				/*	 // for Big Endean
				temp_data[0] = ((BYTE *)&ep932_vdo_settings[params->videosettingindex].e_sync.h_width)[1];
				temp_data[1] = ((BYTE *)&ep932_vdo_settings[params->videosettingindex].e_sync.h_width)[0];
				ep932_reg_write(EP932_H_WIDTH, temp_data, 2);
				*/
				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].e_sync.h_width)[0];   // modify by Eric_Lu for Little Endean
				ep932_reg_write(EP932_H_WIDTH, temp_data, 1);
				//DBG_PRINTK(("[0x83]= 0x%02X\r\n",(int)temp_data[0]));
				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].e_sync.h_width)[1];   // modify by Eric_Lu for Little Endean
				ep932_reg_write(0x84, temp_data, 1);
				//DBG_PRINTK(("[0x84]= 0x%02X\r\n",(int)temp_data[0]));

				temp_data[0] = ep932_vdo_settings[params->videosettingindex].e_sync.v_dly;
				ep932_reg_write(EP932_V_DELAY, temp_data, 1);
				//DBG_PRINTK(("[0x85]= 0x%02X\r\n",(int)temp_data[0]));

				temp_data[0] = ep932_vdo_settings[params->videosettingindex].e_sync.v_width;
				ep932_reg_write(EP932_V_WIDTH, temp_data, 1);
				//DBG_PRINTK(("[0x86]= 0x%02X\r\n",(int)temp_data[0]));

				/*	 // for Big Endean
				temp_data[0] = ((BYTE *)&ep932_vdo_settings[params->videosettingindex].e_sync.v_ofst)[1];
				temp_data[1] = ((BYTE *)&ep932_vdo_settings[params->videosettingindex].e_sync.v_ofst)[0];
				ep932_reg_write(EP932_V_OFF_SET, temp_data, 2);
				*/
				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].e_sync.v_ofst)[0];   // modify by Eric_Lu for Little Endean
				ep932_reg_write(EP932_V_OFF_SET, temp_data, 1);
				//DBG_PRINTK(("[0x87]= 0x%02X\r\n",(int)temp_data[0]));
				temp_data[0] = ((unsigned char *)&ep932_vdo_settings[params->videosettingindex].e_sync.v_ofst)[1];   // modify by Eric_Lu for Little Endean
				ep932_reg_write(0x88, temp_data, 1);
				//DBG_PRINTK(("[0x88]= 0x%02X\r\n",(int)temp_data[0]));

				DBG_PRINTK(("Update E_S.nC params 0x%02X", (unsigned short)ep932_vdo_settings[params->videosettingindex].e_sync.ctl));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].e_sync.h_dly));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].e_sync.h_width));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].e_sync.v_dly));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].e_sync.v_width));
				DBG_PRINTK((", %u", (unsigned short)ep932_vdo_settings[params->videosettingindex].e_sync.v_ofst));
				DBG_PRINTK(("\r\n"));


				for(i=0x80; i<=0x88; i++)
				{
					ep932_reg_read(i, temp_data, 1);
					DBG_PRINTK(("EP932_reg[0x%02X]=0x%02X\r\n",(int)i,(int)temp_data[0]));
				}


				// Regular VSO_POL, HSO_POL
				if(ep932_vdo_settings[params->videosettingindex].hvres_type.hvpol & VNEGHPOS) { // VNeg?
					cache_ep932_de_control |= EP932_DE_CONTROL__VSO_POL;
				}
				else {
					cache_ep932_de_control &= ~EP932_DE_CONTROL__VSO_POL;
				}
				if(ep932_vdo_settings[params->videosettingindex].hvres_type.hvpol & VPOSHNEG) { // HNeg?
					cache_ep932_de_control |= EP932_DE_CONTROL__HSO_POL;
				}
				else {
					cache_ep932_de_control &= ~EP932_DE_CONTROL__HSO_POL;
				}
			}
			else {
				DBG_PRINTK(("ERROR:.videocode overflow E_S.nC table\r\n"));
			}
			break;
	}
	ep932_reg_write(EP932_DE_CONTROL, &cache_ep932_de_control, 1);

	// Pixel Repetition
	ep932_reg_read(EP932_PIXEL_REPETITION_CONTROL, temp_data, 1);
	temp_data[0] &= ~EP932_PIXEL_REPETITION_CONTROL__PR;
	if(params->videosettingindex < ep932_vdo_settings_max) {
		temp_data[0] |= ep932_vdo_settings[params->videosettingindex].ar_pr & 0x03;
	}
	ep932_reg_write(EP932_PIXEL_REPETITION_CONTROL, temp_data, 1);

	// Color Space
	switch(params->formatin) {
		default:
	 	case COLORFORMAT_RGB:
			ep932_reg_clear_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__YCC_IN | EP932_GENERAL_CONTROL_4__422_IN);
			DBG_PRINTK(("Set to RGB In\r\n"));
			break;
	 	case COLORFORMAT_YCC444:
			ep932_reg_set_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__YCC_IN);
			ep932_reg_clear_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__422_IN);
			DBG_PRINTK(("Set to YCC444 In\r\n"));
			break;
	 	case COLORFORMAT_YCC422:
			ep932_reg_set_bit(EP932_GENERAL_CONTROL_4, EP932_GENERAL_CONTROL_4__YCC_IN | EP932_GENERAL_CONTROL_4__422_IN);
			DBG_PRINTK(("Set to YCC422 In\r\n"));
			break;
	}
	switch(params->formatout) {
		default:
	 	case COLORFORMAT_RGB:
			// Set to RGB
			if(params->videosettingindex < EP932_VDO_SETTINGS_IT_START) { // CE Timing
				ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__YCC_OUT | EP932_COLOR_SPACE_CONTROL__422_OUT);
				ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__YCC_RANGE); // Output limit range RGB
			}
			else { // IT Timing
				ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__YCC_OUT | EP932_COLOR_SPACE_CONTROL__422_OUT | EP932_COLOR_SPACE_CONTROL__YCC_RANGE);
			}
			DBG_PRINTK(("Set to RGB Out\r\n"));
			break;

	 	case COLORFORMAT_YCC444:
			// Set to YCC444
			ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__YCC_OUT);
			ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__422_OUT);
			DBG_PRINTK(("Set to YCC444 Out\r\n"));
			break;
	 	case COLORFORMAT_YCC422:
			// Set to YCC422
			ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__YCC_OUT | EP932_COLOR_SPACE_CONTROL__422_OUT);
			DBG_PRINTK(("Set to YCC422 Out\r\n"));
			break;
	}

	// Color Space
	switch(params->colorspace) {
		default:
	 	case COLORSPACE_601:
			// Set to 601
			ep932_reg_clear_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__COLOR);
			DBG_PRINTK(("Set to 601 color definition\r\n"));
			break;

	 	case COLORSPACE_709:
			// Set to 709
			ep932_reg_set_bit(EP932_COLOR_SPACE_CONTROL, EP932_COLOR_SPACE_CONTROL__COLOR);
			DBG_PRINTK(("Set to 709 color definition\r\n"));
			break;
	}

	//
	// Update AVI Info Frame
	//
	// Read AVI Info Frame
	memset(temp_data, 0x00, 14);
	//temp_data[1] &= 0x60;
	switch(params->formatout) {
		default:
	 	case COLORFORMAT_RGB:
			// Set AVI Info Frame to RGB
			temp_data[1] |= 0x00; // RGB
			break;

	 	case COLORFORMAT_YCC444:
			// Set AVI Info Frame to RGB
			temp_data[1] |= 0x40; // YCC 444
			break;
	 	case COLORFORMAT_YCC422:
			// Set AVI Info Frame to RGB
			temp_data[1] |= 0x20; // YCC 422
			break;
	}
	temp_data[1] |= 0x10; // Active Format Information
	//temp_data[2] &= 0xC0;

//	temp_data[1] |= 0x02; // underscan
//	temp_data[1] |= 0x01; // overderscan
	
	switch(params->colorspace) {
		default:
	 	case COLORSPACE_601:
			// Set AVI Info Frame to 601
			temp_data[2] |= 0x40;
			break;

	 	case COLORSPACE_709:
			// Set AVI Info Frame to 709
			temp_data[2] |= 0x80;
			break;
	}
	//temp_data[2] &= 0x30;
	if(params->videosettingindex < ep932_vdo_settings_max) {
		temp_data[2] |= ep932_vdo_settings[params->videosettingindex].ar_pr & 0x30;
	}
	//temp_data[2] &= 0x0F;
	temp_data[2] |= params->afarate & 0x0F;
	if(params->videosettingindex < EP932_VDO_SETTINGS_IT_START) {
		temp_data[4] |= ep932_vdo_settings[params->videosettingindex].videocode;
	}
	if(params->videosettingindex < ep932_vdo_settings_max) {
		temp_data[5] |= (ep932_vdo_settings[params->videosettingindex].ar_pr & 0x0C) >> 2;
	}

	// Write AVI Info Frame
	temp_data[0] = 0x91;
	for(i=1; i<6; ++i) {
		temp_data[0] += temp_data[i];
	}
	temp_data[0] = ~(temp_data[0] - 1);
	ep932_reg_write(EP932_AVI_PACKET, temp_data, 14);

	DBG_PRINTK(("AVI Info: "));
	for(i=0; i<6; ++i) {
		DBG_PRINTK(("0x%02x, ", (int)temp_data[i] ));
	}
	DBG_PRINTK(("\r\n"));

	//
	// Enable Video
	//
	ep932_reg_set_bit(EP932_IIS_CONTROL, EP932_IIS_CONTROL__AVI_EN);
}

void hdmi_tx_audio_config(pado_params params)
{
	int i;
	unsigned char n_cts_index;
	unsigned long n_value, cts_value;
	adsfreq final_frequency;
	unsigned char final_ads_rate;

	DBG_PRINTK(("\r\nStart Tx Audio Config\r\n"));

	//
	// Audio Settings
	//
	// Update WS_M, WS_POL, SCK_POL
	ep932_reg_read(EP932_IIS_CONTROL, temp_data, 1);
	temp_data[0] &= ~0x07;
	temp_data[0] |= params->interface & 0x07;
	ep932_reg_write(EP932_IIS_CONTROL, temp_data, 1);

	// Update Channel Status
	if(params->interface & 0x08) { // IIS

		temp_data[0] = 0;
		// Update Flat | IIS
		temp_data[0] |= ep932_ado_settings[params->channelnumber].flat;
		// Update Channel.number
		if(params->channelnumber > 1) {	// 3 - 8 channel
			temp_data[0] |= EP932_PACKET_CONTROL__LAYOUT;
		}
		ep932_reg_write(EP932_PACKET_CONTROL, temp_data, 1); // Clear IIS
		temp_data[0] |= EP932_PACKET_CONTROL__IIS;
		ep932_reg_write(EP932_PACKET_CONTROL, temp_data, 1); // Set   IIS
		

		// Downsample Convert
		final_ads_rate = params->adsrate;
		switch(params->adsrate) {
			default:
			case 0: // Bypass
				DBG_PRINTK(("Audio ADS = 0\r\n"));
				final_ads_rate = 0;
				final_frequency = params->inputfrequency;
				break;
			case 1: // 1/2
				DBG_PRINTK(("Audio ADS = 1_2\r\n"));
				switch(params->inputfrequency) {
					default: // Bypass
						//DBG_PRINTK(("Audio ADS = 0\r\n"));
						final_ads_rate = 0;
						final_frequency = params->inputfrequency;
						break;
					case ADSFREQ_88200HZ:
						final_frequency = ADSFREQ_44100HZ;
						break;
					case ADSFREQ_96000HZ:
						final_frequency = ADSFREQ_48000HZ;
						break;
					case ADSFREQ_176400HZ:
						final_frequency = ADSFREQ_88200HZ;
						break;
					case ADSFREQ_192000HZ:
						final_frequency = ADSFREQ_96000HZ;
						break;
				}
				break;
			case 2: // 1/3
				//DBG_PRINTK(("Audio ADS = 1_3\r\n"));
				switch(params->inputfrequency) {
					default: // Bypass
						//DBG_PRINTK(("Audio ADS = 0\r\n"));
						final_ads_rate = 0;
						final_frequency = params->inputfrequency;
						break;
					case ADSFREQ_96000HZ:
						final_frequency = ADSFREQ_32000HZ;
						break;
				}
				break;
			case 3: // 1/4
				//DBG_PRINTK(("Audio ADS = 1_4\r\n"));
				switch(params->inputfrequency) {
					default: // Bypass
						//DBG_PRINTK(("Audio ADS = 0\r\n"));
						final_ads_rate = 0;
						final_frequency = params->inputfrequency;
						break;
					case ADSFREQ_176400HZ:
						final_frequency = ADSFREQ_44100HZ;
						break;
					case ADSFREQ_192000HZ:
						final_frequency = ADSFREQ_48000HZ;
						break;
				}
				break;
		}

		// Update Down Sample ADSRate
		ep932_reg_read(EP932_PIXEL_REPETITION_CONTROL, temp_data, 1);
		temp_data[0] &= ~0x30;
		temp_data[0] |= (final_ads_rate << 4) & 0x30;
		ep932_reg_write(EP932_PIXEL_REPETITION_CONTROL, temp_data, 1);


		// Set Channel Status
		memset(temp_data, 0x00, 5);
		temp_data[0] = (params->nocopyright)? 0x04:0x00;
		temp_data[1] = 0x00; 			// Category code ??
		temp_data[2] = 0x00; 			// Channel number ?? | Source number ??
		temp_data[3] = final_frequency; 	// Clock accuracy ?? | Sampling frequency
		temp_data[4] = 0x01; 			// Original sampling frequency ?? | Word length ??
		ep932_reg_write(EP932_CHANNEL_STATUS, temp_data, 5);

		DBG_PRINTK(("CS Info: "));
		for(i=0; i<5; ++i) {
			DBG_PRINTK(("0x%02X, ", (int)temp_data[i] ));
		}
		DBG_PRINTK(("\r\n"));

		ep932_reg_set_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__CS_M);
	}
	else { // SPIDIF

		ep932_reg_set_bit(EP932_PACKET_CONTROL, EP932_PACKET_CONTROL__IIS);
		ep932_reg_clear_bit(EP932_PACKET_CONTROL, EP932_PACKET_CONTROL__FLAT3 | EP932_PACKET_CONTROL__FLAT2 | EP932_PACKET_CONTROL__FLAT1 | EP932_PACKET_CONTROL__FLAT0 |
			EP932_PACKET_CONTROL__IIS | EP932_PACKET_CONTROL__LAYOUT);

		//.no Downsample
		final_ads_rate = 0;
		final_frequency = params->inputfrequency;

		// Disable Down Sample and Bypass Channel Status
		ep932_reg_clear_bit(EP932_PIXEL_REPETITION_CONTROL, EP932_PIXEL_REPETITION_CONTROL__ADSR | EP932_PIXEL_REPETITION_CONTROL__CS_M);

		params->channelnumber = 0;
	}

	// Set.cts.n
	if(params->videosettingindex < ep932_vdo_settings_max) {
		n_cts_index = ep932_vdo_settings[params->videosettingindex].pix_freq_type;
		if(ep932_vdo_settings[params->videosettingindex].hvres_type.vprd % 500) { // 59.94/60 HZ
			n_cts_index += params->vfs;
			DBG_PRINTK(("n_cts_index Shift %d\r\n", (int)params->vfs));
		}
	}
	else {
		DBG_PRINTK(("Use default n_cts_index\r\n"));
		n_cts_index = PIX_FREQ_25200KHZ;
	}
	switch(final_frequency) {

		default:
		case ADSFREQ_32000HZ:
			DBG_PRINTK(("Set to 32KHZ"));
			n_value = n_cts_32k[n_cts_index].n;
			cts_value = n_cts_32k[n_cts_index].cts;
			break;
		case ADSFREQ_44100HZ:
			DBG_PRINTK(("Set to 44.1KHZ"));
			n_value = n_cts_44k1[n_cts_index].n;
			cts_value = n_cts_44k1[n_cts_index].cts;
			break;
		case ADSFREQ_48000HZ:
			DBG_PRINTK(("Set to 48KHZ"));
			n_value = n_cts_48k[n_cts_index].n;
			cts_value = n_cts_48k[n_cts_index].cts;
			break;
		case ADSFREQ_88200HZ:
			DBG_PRINTK(("Set to 88.2KHZ"));
			n_value = n_cts_44k1[n_cts_index].n * 2;
			cts_value = n_cts_44k1[n_cts_index].cts * 2;
			break;
		case ADSFREQ_96000HZ:
			DBG_PRINTK(("Set to 96KHZ"));
			n_value = n_cts_48k[n_cts_index].n * 2;
			cts_value = n_cts_48k[n_cts_index].cts * 2;
			break;
		case ADSFREQ_176400HZ:
			DBG_PRINTK(("Set to 176.4KHZ"));
			n_value = n_cts_44k1[n_cts_index].n * 4;
			cts_value = n_cts_44k1[n_cts_index].cts * 4;
			break;
		case ADSFREQ_192000HZ:
			DBG_PRINTK(("Set to 192KHZ")); n_value = n_cts_48k[n_cts_index].n * 4;
			cts_value = n_cts_48k[n_cts_index].cts * 4;
			break;
	}

	DBG_PRINTK((",n[%d]=%lu(0x%lx)", n_cts_index, n_value, n_value));
	DBG_PRINTK((",cts=%lu(0x%lx) \r\n", cts_value, cts_value));

	temp_data[0] = cts_value>>16;
	ep932_reg_write(EP932_CTS, temp_data, 1);
	temp_data[0] = cts_value>>8;
	ep932_reg_write(0x61, temp_data, 1);
	temp_data[0] = cts_value;
	ep932_reg_write(0x62, temp_data, 1);

	temp_data[0] = n_value>>16;
	ep932_reg_write(EP932_N, temp_data, 1);
	temp_data[0] = n_value>>8;
	ep932_reg_write(0x64, temp_data, 1);
	temp_data[0] = n_value;
	ep932_reg_write(0x65, temp_data, 1);

// read for verify

	ep932_reg_read(EP932_CTS, temp_data, 1);
	DBG_PRINTK(("EP932_cts_0(Reg addr 0x60) = 0x%02X\r\n",(int)temp_data[0]));
	ep932_reg_read(0x61, temp_data, 1);
	DBG_PRINTK(("EP932_cts_1(Reg addr 0x61) = 0x%02X\r\n",(int)temp_data[0]));
	ep932_reg_read(0x62, temp_data, 1);
	DBG_PRINTK(("EP932_cts_2(Reg addr 0x62) = 0x%02X\r\n",(int)temp_data[0]));

	ep932_reg_read(EP932_N, temp_data, 1);
	DBG_PRINTK(("EP932_n_0(Reg addr 0x63) = 0x%02X\r\n",(int)temp_data[0]));
	ep932_reg_read(0x64, temp_data, 1);
	DBG_PRINTK(("EP932_n_1(Reg addr 0x64) = 0x%02X\r\n",(int)temp_data[0]));
	ep932_reg_read(0x65, temp_data, 1);
	DBG_PRINTK(("EP932_n_2(Reg addr 0x65) = 0x%02X\r\n",(int)temp_data[0]));

	//
	// Update ADO Info Frame
	//
	// Set Default ADO Info Frame
	memset(temp_data, 0x00, 6);

	// Overwrite ADO Info Frame
	temp_data[1] = params->channelnumber;
	temp_data[4] = ep932_ado_settings[params->channelnumber].speakermapping;

	// Write ADO Info Frame back
	temp_data[0] = 0x8F;
	for(i=1; i<6; ++i) {
		temp_data[0] += temp_data[i];
	}
	temp_data[0] = ~(temp_data[0] - 1);
	ep932_reg_write(EP932_ADO_PACKET, temp_data, 6);

	DBG_PRINTK(("ADO Info: "));
	for(i=0; i<6; ++i) {
		DBG_PRINTK(("0x%02x, ", (int)temp_data[i] ));
	}
	DBG_PRINTK(("\r\n"));

	ep932_reg_set_bit(EP932_IIS_CONTROL, EP932_IIS_CONTROL__ACR_EN | EP932_IIS_CONTROL__ADO_EN | EP932_IIS_CONTROL__AUDIO_EN);
	//DBG_PRINTK(("66666666666666666666666666666^^^^^^^^^^^^^^^\r\n"));
}

//--------------------------------------------------------------------------------------------------
//
// Hardware Interface
//
smbus_status key_read(unsigned char byteaddr, void *data, unsigned int size)
{
	return iic_read(iic_key_addr, byteaddr, data, size);
}

smbus_status key_write(unsigned char byteaddr, void *data, unsigned int size)
{
	return iic_write(iic_key_addr, byteaddr, data, size);
}

smbus_status ep932_reg_read(unsigned char byteaddr, unsigned char *data, unsigned int size)
{
	return iic_read(iic_ep932_addr, byteaddr, data, size);
}

smbus_status ep932_reg_write(unsigned char byteaddr, unsigned char *data, unsigned int size)
{
	//DBG_PRINTK(("ep932_reg_write 0x%02X, 0x%02X\r\n",(int)byteaddr,(int)data[0]));
	return iic_write(iic_ep932_addr, byteaddr, data, size);
}

smbus_status ep932_reg_set_bit(unsigned char byteaddr, unsigned char bitmask)
{
	iic_read(iic_ep932_addr, byteaddr, temp_data, 1);

	// Write back to Reg Reg_Addr
	temp_data[0] |= bitmask;

	return iic_write(iic_ep932_addr, byteaddr, temp_data, 1);
}

smbus_status ep932_reg_clear_bit(unsigned char byteaddr, unsigned char bitmask)
{
	iic_read(iic_ep932_addr, byteaddr, temp_data, 1);

	// Write back to Reg Reg_Addr
	temp_data[0] &= ~bitmask;

	return iic_write(iic_ep932_addr, byteaddr, temp_data, 1);
}



smbus_status iic_write(unsigned char iic_addr, unsigned char byteaddr, unsigned char *data, unsigned int size)
{
	int result = 1;
	int trytime=10;
	int i;
	//DBG_PRINTK(("iic_write 0x%02X, 0x%02X\r\n",(int)byteaddr,(int)data[0]));
	do
	{
	result = jz_i2c_ep932_write(byteaddr, data, size);
	trytime--;
	if(trytime==0)
		break;
	}while(result);
	if(result != 0)
	{
		DBG_PRINTK(("EP932M iic_write error : 0x%02X, 0x%02X, (%d):",(int)iic_addr,(int)byteaddr, (int)size));
		printk("EP932M iic_write error : 0x%02X, 0x%02X, (%d):",(int)iic_addr,(int)byteaddr, (int)size);
		for(i=0; i<size; i++)
		{
			DBG_PRINTK((" 0x%02X",(int)data[i]));
		}
		DBG_PRINTK(("\r\n"));
	}
	
	return result;

}

smbus_status iic_read(unsigned char iic_addr, unsigned char byteaddr, unsigned char *data, unsigned int size)
{
	int result = 1;

//	result = TLGI2C_ReadReg_EP932M(iic_addr, byteaddr, data, size);
	result = jz_i2c_ep932_read(byteaddr, data, size);
	if(result != 0)
	{
		DBG_PRINTK(("EP932M iic_read error : 0x%02X, 0x%02X, 0x%02X, %d\r\n",(int)iic_addr,(int)byteaddr,(int)data[0],size));
		printk("EP932M iic_read error : 0x%02X, 0x%02X, 0x%02X, %d\r\n",(int)iic_addr,(int)byteaddr,(int)data[0],size);
	}

	return result;

}
