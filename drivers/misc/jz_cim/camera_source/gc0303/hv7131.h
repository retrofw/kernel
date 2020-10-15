#ifndef __HV7131_H__
#define __HV7131_H__
/*
 * hv7131 registers
 */
#define DEVID    0x00 /* Dev ID */
#define SCTRA    0x01 /* Sensor Control A */
#define SCTRB    0x02 /* Sensor Control B */
#define OUTIV    0x03 /* Output Inversion */

#define RSAU     0x10 /* Row Start Address Upper */
#define RSAL     0x11 /* Row Start Address Lower */
#define CSAU     0x12 /* Col Start Address Upper */
#define CSAL     0x13 /* Col Start Address Lower */
#define WIHU     0x14 /* Window Height Upper */
#define WIHL     0x15 /* Window Height Lower */
#define WIWU     0x16 /* Window Width Upper */
#define WIWL     0x17 /* Window Width Lower */

#define HBLU     0x20 /* HBLANK Time Upper */
#define HBLL     0x21 /* HBLANK Time Lower */
#define VBLU     0x22 /* VBLANK Time Upper */
#define VBLL     0x23 /* VBLANK Time Lower */
#define INTH     0x25 /* Integration Time High */
#define INTM     0x26 /* Integration Time Middle */
#define INTL     0x27 /* Integration Time Low */

#define PAG      0x30 /* Pre-amp Gain */
#define RCG      0x31 /* Red Color Gain */
#define GCG      0x32 /* Green Color Gain */
#define BCG      0x33 /* Blue Color Gain */
#define ACTRA    0x34 /* Analog Bias Control A */
#define ACTRB    0x35 /* Analog Bias Control B */

#define BLCTH    0x40 /* Black Level Threshod */
#define ORedI    0x41 /* Initial ADC Offset Red */
#define OGrnI    0x42 /* Initial ADC Offset Green */
#define OBluI    0x43 /* Initial ADC Offset Blue */

#define CMOS_WIDTH      640
#define CMOS_HEIGHT     480

void set_gc0303_i2c_client(struct i2c_client *client);

void sensor_init(int left, int top, int width, int height);
void sensor_power_down_GC303(void);

int SetCMOSGain(int Gain);
int IncCMOSGain(void);
int DecCMOSGain(void);

#endif /* __HV7131_H__ */

