#
# Makefile for the Ingenic JZ4730.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o i2c.o 

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support

obj-$(CONFIG_JZ4730_PMP)	+= board-pmp.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       +=cpufreq.o

# PM support

obj-$(CONFIG_PM_LEGACY)         +=pm.o sleep.o
