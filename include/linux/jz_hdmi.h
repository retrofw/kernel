#ifndef __JZ_HDMI_H__
#define __JZ_HDMI_H__

struct hdmi_it6610_pin_info{
	unsigned int PIN_HPD;
	unsigned int PIN_SYSRSTN;
	unsigned int PIN_INT;
	unsigned int PIN_PCSCL;
	unsigned int PIN_PCSDA;
	unsigned int PIN_MCLK;
	unsigned int PIN_SCK;
	unsigned int PIN_WS;
	unsigned int PIN_I2S0;
	unsigned int PIN_SPDIF;
	unsigned int PIN_PCLK;
	unsigned int PIN_VSYNC;
	unsigned int PIN_HSYNC;
	unsigned int PIN_DE;
};
struct hdmi_it6610_board_info{
	struct hdmi_it6610_pin_info *pin_info;
	unsigned int hpd_connect_active;
	void (*hdmi_board_init)(void);
	void (*hdmi_power_on)(void);
	void (*hdmi_power_off)(void);
	void (*lcd_signal_driving_strengthen_up)(void);
	void (*lcd_signal_driving_strengthen_down)(void);
};

#endif /*__JZ_HDMI_H__*/
