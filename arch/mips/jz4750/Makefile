#
# Makefile for the Ingenic JZ4750.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o i2c.o

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support

obj-$(CONFIG_JZ4750_FUWA)	+= board-fuwa.o
obj-$(CONFIG_JZ4750_APUS)	+= board-apus.o

# PM support

obj-$(CONFIG_PM)         +=pm.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       +=cpufreq.o
