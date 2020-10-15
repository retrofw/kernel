#
# Makefile for jzchar
#
#obj-$(CONFIG_JZCHAR)		+= jzchars.o
# obj-$(CONFIG_JZCHAR)		+= jzchars.o simulate_i2c.o sensor.o gsensor_i2c.o poweroff.o backlight.o
obj-$(CONFIG_JZCHAR)		+= jzchars.o simulate_i2c.o sensor.o gsensor_i2c.o

#obj-$(CONFIG_JZ_TPANEL)		+= jz_ts.o
obj-$(CONFIG_JZ_TPANEL_UCB1400)	+= ucb1400.o
obj-$(CONFIG_JZ_TPANEL_WM9712)	+= wm9712.o
obj-$(CONFIG_JZ_TPANEL_AK4182)	+= ak4182.o
obj-$(CONFIG_JZ_TPANEL_ATA2508)	+= ata2508.o
obj-$(CONFIG_JZ_SADC)		+= sadc.o
obj-$(CONFIG_JZ_TSSI_4760)	+= jz_tssi.o

obj-$(CONFIG_JZ_OW)	+= jz_ow.o
obj-$(CONFIG_JZ_TCSM)	+= tcsm.o

obj-$(CONFIG_JZ_SIMPLE_I2C) += i_i2c.o
obj-$(CONFIG_JZ_GPIO_PM_KEY) += i_gpio_pm_key.o

obj-$(CONFIG_JZ4770_TCU) += jz4770_tcu.o

obj-$(CONFIG_JZ_WIEGAND) += jz_wiegand.o
