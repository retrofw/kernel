#
# Makefile for the Ingenic JZ4740.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o i2c.o

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support

obj-$(CONFIG_JZ4740_PAVO)	+= board-pavo.o
obj-$(CONFIG_JZ4740_LEO)	+= board-leo.o
obj-$(CONFIG_JZ4740_LYRA)	+= board-lyra.o
obj-$(CONFIG_JZ4725_DIPPER)	+= board-dipper.o
obj-$(CONFIG_JZ4720_VIRGO)	+= board-virgo.o

# PM support

obj-$(CONFIG_PM)         +=pm.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       +=cpufreq.o
