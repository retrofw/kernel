#ifndef __JZ4810_TVE_H__
#define __JZ4810_TVE_H__


#define PANEL_MODE_LCD_PANEL	0
#define PANEL_MODE_TVE_PAL	1
#define PANEL_MODE_TVE_NTSC	2

/* TV parameter */
#define TVE_WIDTH_PAL 		720
#define TVE_HEIGHT_PAL 		573
#define TVE_FREQ_PAL 		50
#define TVE_WIDTH_NTSC 		720
#define TVE_HEIGHT_NTSC 	482
#define TVE_FREQ_NTSC 		60


/* Structure for TVE */
struct jz4810tve_info {
	unsigned int ctrl;
	unsigned int frcfg;
	unsigned int slcfg1;
	unsigned int slcfg2;
	unsigned int slcfg3;
	unsigned int ltcfg1;
	unsigned int ltcfg2;
	unsigned int cfreq;
	unsigned int cphase;
	unsigned int cbcrcfg;
	unsigned int wsscr;
	unsigned int wsscfg1;
	unsigned int wsscfg2;
	unsigned int wsscfg3;
};

extern struct jz4810tve_info *jz4810_tve_info;

extern void jz4810tve_enable_tve(void);
extern void jz4810tve_disable_tve(void);

extern void jz4810tve_set_tve_mode( struct jz4810tve_info *tve );
extern void jz4810tve_init( int tve_mode );


#endif	/* __JZ4810_TVE_H__ */
