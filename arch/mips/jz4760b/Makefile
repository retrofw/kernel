#
# Makefile for the Ingenic JZ4760.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o cpm.o proc.o #i2c.o

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support
obj-$(CONFIG_JZ4760_ALTAIR)	+= board-altair.o
obj-$(CONFIG_JZ4760_CYGNUS)	+= board-cygnus.o
obj-$(CONFIG_JZ4760B_CYGNUS)	+= board-cygnus.o
obj-$(CONFIG_JZ4760_LEPUS)	+= board-lepus.o
obj-$(CONFIG_JZ4760B_LEPUS)	+= board-lepus.o
obj-$(CONFIG_JZ4760B_LYNX)	+= board-lynx.o
obj-$(CONFIG_JZ4760B_CARINA)	+= board-carina.o
obj-$(CONFIG_JZ4760B_YEW)	+= board-yew.o
obj-$(CONFIG_JZ4760B_H76W)	+= board-h76w.o
obj-$(CONFIG_JZ4760B_W1)	+= board-w1.o
obj-$(CONFIG_JZ4760B_GM_TEST1)  += board-gm_test1.o
obj-$(CONFIG_JZ4760B_SP_EVK01)  += board-sp_evk01.o

ifneq ($(CONFIG_OLD_I2S_DLV_4760),y)
obj-$(CONFIG_JZ4760B_CARINA)	+= board-carina-audio.o
obj-$(CONFIG_JZ4760B_YEW)	+= board-yew-audio.o
obj-$(CONFIG_JZ4760B_H76W)	+= board-h76w-audio.o
obj-$(CONFIG_JZ4760B_W1)	+= board-w1-audio.o

ifeq ($(CONFIG_I2S_DLV_4760),y)
obj-$(CONFIG_JZ4760B_GM_TEST1)  += board-gm_test1-audio.o
obj-$(CONFIG_JZ4760B_SP_EVK01)  += board-sp_evk01-audio.o
endif

endif

obj-$(CONFIG_JZ4760_F4760)	+= board-f4760.o
obj-$(CONFIG_SOC_JZ4760B)        += fpu.o

# PM support

obj-$(CONFIG_PM)         += pm.o sleep.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       += cpufreq.o

#obj-$(CONFIG_JZ4760_ALTAIR)	+= gpiolib.o
#obj-$(CONFIG_JZ4760B_CYGNUS)	+= gpiolib.o

