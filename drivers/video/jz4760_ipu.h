#ifndef __IPU_H_ME__
#define __IPU_H_ME__

// Bitmap for IPU Control Register
#define BUS_OPT	                    ( 1 << 22 ) 
#define CONF_MODE                   ( 1 << 21 )
#define ADDR_SEL                    ( 1 << 20 )
#define BURST_SEL                   ( 1 << 19 )
#define ZOOM_SEL                    ( 1 << 18 )
#define DFIX_SEL                    ( 1 << 17 )
#define FIELD_SEL                   ( 1 << 16 )
#define FIELD_CONF_EN               ( 1 << 15 )
#define DISP_SEL                    ( 1 << 14 )
#define DPAGE_MAP                   ( 1 << 13 )
#define SPAGE_MAP                   ( 1 << 12 )
#define LCDC_SEL                    ( 1 << 11 )
#define SPKG_SEL                    ( 1 << 10 )
// Bit positions 8 & 9 are reserved
#define IPU_STOP                    ( 1 << 7 )
#define IPU_RST                     ( 1 << 6 )
#define FM_IRQ_EN                   ( 1 << 5 )
#define CSC_EN                      ( 1 << 4 )
#define VRSZ_EN                     ( 1 << 3 )
#define HRSZ_EN                     ( 1 << 2 )
#define IPU_RUN                     ( 1 << 1 )
#define CHIP_EN                     ( 1 << 0 )

// IPU Status Register
#define SIZE_ERR                    ( 1 << 2 )
#define FMT_ERR                     ( 1 << 1 )
#define OUT_END                     ( 1 << 0 )

// Input Geometric Size Register
#define IN_FM_W_BIT				( 16 )
#define IN_FM_W_MASK				( 0xfff << IN_FM_W_BIT )

#define IN_FM_H_BIT				( 0 )
#define IN_FM_H_MASK				( 0xfff << IN_FM_H_BIT )

#define IN_FM_W(val)				((val) << IN_FM_W_BIT)
#define IN_FM_H(val)				((val) << IN_FM_H_BIT)

// Input UV Data Line Stride Register
#define U_S_BIT					( 16 )
#define U_S_MASK				( 0x1fff << U_S_BIT )

#define V_S_BIT					( 0 )
#define V_S_MASK				( 0x1fff << V_S_BIT )

#define U_STRIDE(val)				((val) << U_S_BIT)
#define V_STRIDE(val)				((val) << V_S_BIT)

// Output Geometric Size Register
#define OUT_FM_W_BIT				( 16 )
#define OUT_FM_W_MASK				( 0x1fff << OUT_FM_W_BIT )

#define OUT_FM_H_BIT				( 0 )
#define OUT_FM_H_MASK				( 0x1fff << OUT_FM_H_BIT )

#define OUT_FM_W(val)				((val) << OUT_FM_W_BIT)
#define OUT_FM_H(val)				((val) << OUT_FM_H_BIT)

// Resize Coefficients Table Index Register
#define HE_IDX_W_BIT				( 16 )
#define HE_IDX_W_MASK				( 0x1f << HE_IDX_W_BIT )

#define VE_IDX_H_BIT				( 0 )
#define VE_IDX_H_MASK				( 0x1f << VE_IDX_H_BIT )

//H & V Resize Coefficients Look Up Table Register group
#define W_COEF0_BIT				(6)
#define W_COEF0_MSK				(0x3ff)
#define W_COEF0_MASK				(0x3ff << W_COEF0_BIT)

/*H conf*/
#define H_CONF_BIT				(0)
#define H_CONF_MASK				(1 << H_CONF_BIT)

/*H bi-cube*/
#define HRSZ_OFT_BIT				(1)
#define HRSZ_OFT_MASK				(0x1f << HRSZ_OFT_BIT)

/*H bi-linear*/
#define H_OFT_BIT				(1)
#define H_OFT_MSK				(0x1f)
#define H_OFT_MASK				(0x1f << H_OFT_BIT)

/*V conf*/
#define V_CONF_BIT				(0)
#define V_CONF_MASK				(1 << V_CONF_BIT)

/*V bi-cube*/
#define VRSZ_OFT_BIT				(1)
#define VRSZ_OFT_MASK				(0x1f << VRSZ_OFT_BIT)

/*V bi-linear*/
#define V_OFT_BIT				(1)
#define V_OFT_MSK				(0x1f)
#define V_OFT_MASK				(0x1f << V_OFT_BIT)


// CSC Offset Parameter Register
#define CHROM_OF_W_BIT				( 16 )
#define CHROM_OF_W_MASK				( 0xff << CHROM_OF_W_BIT )
#define CHROM(x)				(x << CHROM_OF_W_BIT)

#define LUMA_OF_H_BIT				( 0 )
#define LUMA_OF_H_MASK				( 0xff << LUMA_OF_H_BIT )
#define LUMA(x)					(x << LUMA_OF_H_BIT)

// Data Format Register
#define RGB_888_OUT_FMT			( 1 << 25 )

#define RGB_OUT_OFT_BIT			( 22 )
#define RGB_OUT_OFT_MASK		( 7 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RGB			( 0 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_RBG			( 1 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GBR			( 2 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_GRB			( 3 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BRG			( 4 << RGB_OUT_OFT_BIT )
#define RGB_OUT_OFT_BGR			( 5 << RGB_OUT_OFT_BIT )

#define OUT_FMT_BIT			( 19 )
#define OUT_FMT_MASK			( 7 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB555			( 0 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB565			( 1 <<  OUT_FMT_BIT )
#define OUT_FMT_RGB888			( 2 <<  OUT_FMT_BIT )
#define OUT_FMT_YUV422			( 3 <<  OUT_FMT_BIT )

#define YUV_PKG_OUT_OFT_BIT		( 16 )
#define YUV_PKG_OUT_OFT_MASK		( 7 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1UY0V		( 0 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y1VY0U		( 1 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY1VY0		( 2 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY1UY0		( 3 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0UY1V		( 4 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_Y0VY1U		( 5 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_UY0VY1		( 6 << YUV_PKG_OUT_OFT_BIT )
#define YUV_PKG_OUT_OFT_VY0UY1		( 7 << YUV_PKG_OUT_OFT_BIT )

#define IN_OFT_BIT			( 2 )
#define IN_OFT_MASK			( 3 << IN_OFT_BIT )
#define IN_OFT_Y1UY0V			( 0 << IN_OFT_BIT )
#define IN_OFT_Y1VY0U			( 1 << IN_OFT_BIT )
#define IN_OFT_UY1VY0			( 2 << IN_OFT_BIT )
#define IN_OFT_VY1UY0			( 3 << IN_OFT_BIT )

#define IN_FMT_BIT			( 0 )
#define IN_FMT_MASK			( 3 << IN_FMT_BIT )
#define IN_FMT_YUV420			( 0 << IN_FMT_BIT )
#define IN_FMT_YUV422			( 1 << IN_FMT_BIT )
#define IN_FMT_YUV444			( 2 << IN_FMT_BIT )
#define IN_FMT_YUV411			( 3 << IN_FMT_BIT )
#define IN_FMT_PKG_RGB565		( 3 << IN_FMT_BIT )


/**
 * Page 425 of developer guide: 
 *   Here n stands for original pixel points, m stands for pixel points after 
 *   resize. For example down-scaling 5:3, n = 5, m = 3. Moreover, m and n are 
 *   prime, that is, for example 8:2 should be converted to 4:1. 
 **/
struct Ration2m
{
	unsigned int ratio;
	int n;  // original pixel points
        int m;  // pixel points after resize
};

typedef struct
{
	unsigned int	coef;
	unsigned short	in_n;
	unsigned short	out_n;
} rsz_lut;

// parameter
// R = 1.164 * (Y - 16) + 1.596 * (cr - 128)    {C0, C1}
// G = 1.164 * (Y - 16) - 0.392 * (cb -128) - 0.813 * (cr - 128)  {C0, C2, C3}
// B = 1.164 * (Y - 16) + 2.017 * (cb - 128)    {C0, C4}
#define YUV_CSC_C0				0x4A8        /* 1.164 * 1024 */
#define YUV_CSC_C1				0x662        /* 1.596 * 1024 */
#define YUV_CSC_C2				0x191        /* 0.392 * 1024 */
#define YUV_CSC_C3				0x341        /* 0.813 * 1024 */
#define YUV_CSC_C4				0x811        /* 2.017 * 1024 */
#define YUV_CSC_CHROM				128
#define YUV_CSC_LUMA				16

struct YuvStride
{
	unsigned int y;
	unsigned int u;
	unsigned int v;
	unsigned int out;
};

typedef struct
{
	unsigned int		ipu_ctrl;				// IPU Control Register
	unsigned int		ipu_data_fmt;				// IPU Data Format Register
	unsigned int		in_width;
	unsigned int		in_height;
	unsigned int		in_bpp;
	unsigned int		out_width;
	unsigned int		out_height;
	unsigned char*		y_buffer;
	unsigned char*          u_buffer;
	unsigned char*		v_buffer;
	unsigned char*		tlb_y_buffer;				// table address
	unsigned char*		tlb_u_buffer;
	unsigned char*		tlb_v_buffer;
	struct YuvStride*	stride;
} img_param_t;

typedef volatile struct _IPU_CTRL2
{
	unsigned int chip_en		:1;
	unsigned int ipu_run		:1;
	unsigned int hrsz_en		:1;
	unsigned int vrsz_en		:1;
	unsigned int csc_en		:1;
	unsigned int fm_irq_en		:1;
	unsigned int ipu_rst		:1;
	unsigned int ipu_stop		:1;
	unsigned int rsv0		:2;
	unsigned int spkg_sel		:1;
	unsigned int lcdc_sel		:1;
	unsigned int spage_map		:1;
	unsigned int dpage_map		:1;
	unsigned int disp_sel		:1;
	unsigned int field_conf_en	:1;
	unsigned int field_sel		:1;
	unsigned int dfix_sel		:1;
	unsigned int zoom_sel		:1;
	unsigned int burst_sel		:1;
	unsigned int add_sel		:1;
	unsigned int conf_mode		:1;
	unsigned int bus_opt		:1;
	unsigned int rsv1		:9;
} IPU_CTRL2, *PIPU_CTRL2;

typedef volatile struct _IPU_D_FMT2
{
	unsigned int in_fmt		:2;
	unsigned int in_oft		:2;
	unsigned int in_rgb_fmt		:2;
	unsigned int rsv0		:10;
	unsigned int pkg_out_oft	:3;
	unsigned int out_fmt		:3;
	unsigned int rgb_out_oft	:3;
	unsigned int out_ftm_24b	:1;
	unsigned int rsv1		:6;
} IPU_DFMT2, *PIPU_DFMT2;

#define IPU_LUT_LEN 32
#define IPU_RATIO_MUL 100000

#endif

