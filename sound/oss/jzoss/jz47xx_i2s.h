#ifndef __JZ47XX_I2S_H__
#define __JZ47XX_I2S_H__

extern void jz47xx_i2s_dump_regs(const char *str);
extern int jz47xx_i2s_init(void);
extern int jz47xx_i2s_set_width(int mode, int width);
extern int jz47xx_i2s_set_channels(int mode, int channels);
extern void jz47xx_i2s_tx_ctrl(int on);
extern void jz47xx_i2s_rx_ctrl(int on);

/* NOTE: when use internal codec, nothing to do with sample rate here.
 * 	if use external codec and bit clock is provided by I2S controller, set clock rate here!!!
 */
#define jz47xx_i2s_set_rate(mode, rate) do{ }while(0)


#endif /* __JZ47XX_I2S_H__ */
