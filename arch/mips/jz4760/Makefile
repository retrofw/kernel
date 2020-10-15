#
# Makefile for the Ingenic JZ4760.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o cpm.o #i2c.o

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support
obj-$(CONFIG_JZ4760_ALTAIR)	+= board-altair.o
obj-$(CONFIG_JZ4760_CYGNUS)	+= board-cygnus.o
obj-$(CONFIG_JZ4760_LEPUS)	+= board-lepus.o
obj-$(CONFIG_JZ4760_HTB80)	+= board-htb80.o

obj-$(CONFIG_JZ4760_F4760)	+= board-f4760.o
obj-$(CONFIG_SOC_JZ4760)        += fpu.o

# PM support

obj-$(CONFIG_PM)         += pm.o sleep.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       += cpufreq.o

#obj-$(CONFIG_JZ4760_ALTAIR)	+= gpiolib.o

