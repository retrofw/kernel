#
# Makefile for the Ingenic JZ4770.
#

# Object file lists.

obj-y += prom.o irq.o time.o reset.o setup.o dma.o \
	platform.o cpm.o

obj-$(CONFIG_PROC_FS)		+= proc.o

# board specific support
obj-$(CONFIG_JZ4770_F4770)	+= board-f4770.o
obj-$(CONFIG_JZ4770_PISCES)	+= board-pisces.o
obj-$(CONFIG_JZ4770_TJ70)	+= board-tj70.o
obj-$(CONFIG_SOC_JZ4770)        += fpu.o

# PM support

obj-$(CONFIG_PM)         += pm.o sleep.o

# CPU Frequency scaling support

obj-$(CONFIG_CPU_FREQ_JZ)       += cpufreq.o

