#ifndef __JZ_CHARS_H__
#define __JZ_CHARS_H__

#include <linux/list.h>
#include <linux/fs.h>

#define JZ_CHAR_MAJOR		238

#define UPRT_MINOR		0  // Micro printer
#define CIM_MINOR		1  // Camera interface module
#define TPANEL_MINOR		2  // Touchpanel
#define KEYPAD_MINOR		3  // Keypad
#define MEMCARD_MINOR		4  // Memory card
#define MAGCARD_MINOR		5  // Magcard
#define VFD_MINOR		6  // VFD
#define POWERFAIL_MINOR		7  // Powerfail
#define EJTAG_MINOR		8  // EJTAG emulation
#define REMR0_MINOR		9  // Remote output receive 0
#define REMR1_MINOR		10 // Remote output receive 1
#define USPI_MINOR		11 // Ultra-speed SPI device
#define SADC_MINOR		12 // SAR-ADC
#define SLCD_MINOR		13 // Smart LCD

// 32 to 47 are reserved for SCC
#define SCC_MINOR		32
// 48 to 63 are reserved for Camera sensor
#define SENSOR_MINOR		48
// 64 to 71 are for EEPROM
#define EEPROM_MINOR_BASE       64
// 72 for OWI
#define OW_MINOR                72
// 73 for TCSM_MINOR
#define TCSM_MINOR              73

typedef struct {
	struct list_head list;
	char *name;
	struct file_operations *fops;
	void *private;
	unsigned short dev_minor;
} jz_char_dev_t;

extern int jz_register_chrdev(unsigned char minor, const char *name,
			      struct file_operations *fops, void * private);
extern int jz_unregister_chrdev(unsigned char minor, const char *name);

#endif /*  __JZ_CHARS_H__  */
