
#ifndef __NMI_I2C_H__
#define __NMI_I2C_H__

extern void nmi_bus_write(u32 adr, u8 *b, u32 sz);
extern void nmi_bus_read(u32 adr, u8 *b, u32 sz);

#endif
