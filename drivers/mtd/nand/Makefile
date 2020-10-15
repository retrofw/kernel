#
# linux/drivers/nand/Makefile
#

obj-$(CONFIG_MTD_NAND)			+= nand.o nand_ecc.o
obj-$(CONFIG_MTD_NAND_IDS)		+= nand_ids.o

obj-$(CONFIG_MTD_NAND_CAFE)		+= cafe_nand.o
obj-$(CONFIG_MTD_NAND_SPIA)		+= spia.o
obj-$(CONFIG_MTD_NAND_AMS_DELTA)	+= ams-delta.o
obj-$(CONFIG_MTD_NAND_AUTCPU12)		+= autcpu12.o
obj-$(CONFIG_MTD_NAND_EDB7312)		+= edb7312.o
obj-$(CONFIG_MTD_NAND_AU1550)		+= au1550nd.o
obj-$(CONFIG_MTD_NAND_BF5XX)		+= bf5xx_nand.o
obj-$(CONFIG_MTD_NAND_PPCHAMELEONEVB)	+= ppchameleonevb.o
obj-$(CONFIG_MTD_NAND_S3C2410)		+= s3c2410.o
obj-$(CONFIG_MTD_NAND_DAVINCI)		+= davinci_nand.o
obj-$(CONFIG_MTD_NAND_DISKONCHIP)	+= diskonchip.o
obj-$(CONFIG_MTD_NAND_H1900)		+= h1910.o
obj-$(CONFIG_MTD_NAND_RTC_FROM4)	+= rtc_from4.o
obj-$(CONFIG_MTD_NAND_SHARPSL)		+= sharpsl.o
obj-$(CONFIG_MTD_NAND_TS7250)		+= ts7250.o
obj-$(CONFIG_MTD_NAND_NANDSIM)		+= nandsim.o
obj-$(CONFIG_MTD_NAND_CS553X)		+= cs553x_nand.o
obj-$(CONFIG_MTD_NAND_NDFC)		+= ndfc.o
obj-$(CONFIG_MTD_NAND_ATMEL)		+= atmel_nand.o
obj-$(CONFIG_MTD_NAND_GPIO)		+= gpio.o
obj-$(CONFIG_MTD_NAND_OMAP2) 		+= omap2.o
obj-$(CONFIG_MTD_NAND_CM_X270)		+= cmx270_nand.o
obj-$(CONFIG_MTD_NAND_BASLER_EXCITE)	+= excite_nandflash.o
obj-$(CONFIG_MTD_NAND_PXA3xx)		+= pxa3xx_nand.o
obj-$(CONFIG_MTD_NAND_TMIO)		+= tmio_nand.o
obj-$(CONFIG_MTD_NAND_PLATFORM)		+= plat_nand.o
obj-$(CONFIG_MTD_ALAUDA)		+= alauda.o
obj-$(CONFIG_MTD_NAND_PASEMI)		+= pasemi_nand.o
obj-$(CONFIG_MTD_NAND_ORION)		+= orion_nand.o
obj-$(CONFIG_MTD_NAND_FSL_ELBC)		+= fsl_elbc_nand.o
obj-$(CONFIG_MTD_NAND_FSL_UPM)		+= fsl_upm.o
obj-$(CONFIG_MTD_NAND_SH_FLCTL)		+= sh_flctl.o
obj-$(CONFIG_MTD_NAND_MXC)		+= mxc_nand.o
obj-$(CONFIG_MTD_NAND_SOCRATES)		+= socrates_nand.o
obj-$(CONFIG_MTD_NAND_TXX9NDFMC)	+= txx9ndfmc.o
obj-$(CONFIG_MTD_NAND_JZ4730)		+= jz4730_nand.o
obj-$(CONFIG_MTD_NAND_JZ4740)		+= jz4740_nand.o
obj-$(CONFIG_MTD_NAND_JZ4750)		+= jz4750_nand.o
ifeq ($(CONFIG_MTD_UBI),y)
obj-$(CONFIG_MTD_NAND_JZ4760)		+= jz4760_nand_ubi.o
obj-$(CONFIG_MTD_NAND_JZ4760B)		+= jz4760_nand_ubi.o
obj-$(CONFIG_MTD_NAND_JZ4770)		+= jz4770_nand_ubi.o
else
obj-$(CONFIG_MTD_NAND_JZ4760)		+= jz4760_nand.o
obj-$(CONFIG_MTD_NAND_JZ4760B)		+= jz4760b_nand.o
obj-$(CONFIG_MTD_NAND_JZ4770)		+= jz4770_nand.o
endif
obj-$(CONFIG_MTD_NAND_JZ4810)		+= jz4810_nand.o

ifeq ($(CONFIG_MTD_UBI),y)
nand-objs := nand_base_ubi.o nand_bbt_ubi.o
else
nand-objs := nand_base.o nand_bbt.o
endif

