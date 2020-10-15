#
# Makefile for the Ingenic JZ4750D.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o i2c.o

obj-$(CONFIG_PROC_FS)		+= proc.o
obj-$(CONFIG_GPIOLIB)		+= gpiolib.o
# board specific support

obj-$(CONFIG_JZ4750D_FUWA1)	+= board-fuwa1.o
obj-$(CONFIG_JZ4750D_CETUS)	+= board-cetus.o

# PM support

obj-$(CONFIG_PM)         +=pm.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       +=cpufreq.o
