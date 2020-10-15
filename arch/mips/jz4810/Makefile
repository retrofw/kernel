#
# Makefile for the Ingenic JZ4810.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o cpm_fake.o #i2c.o

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support
obj-$(CONFIG_JZ4810_F4810)	+= board-f4810.o
obj-$(CONFIG_SOC_JZ4810)        += fpu.o

# PM support

obj-$(CONFIG_PM)         += pm.o sleep.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       += cpufreq.o

#obj-$(CONFIG_JZ4760_ALTAIR)	+= gpiolib.o

