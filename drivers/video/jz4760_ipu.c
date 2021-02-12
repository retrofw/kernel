/*
 * linux/drivers/video/jz4760_ipu.c -- 4760 IPU controller for RetroFW
 *
 * Copyright (C) 2020, Jerason Banes and PingFlood (aka 0xdc)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/irq.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#if defined(CONFIG_FB_JZ4750_LCD)
#include "jz4750_lcd.h"
#elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
#include "jz4770_lcd.h"
#elif defined(CONFIG_FB_JZ4760_LCD)
#include "jz4760_lcd.h"
#endif

#if defined CONFIG_JZ4750
#include <asm/mach-jz4750/jz4750.h>
#include <asm/mach-jz4750/regs.h>
#include <asm/mach-jz4750/ops.h>
#elif defined CONFIG_SOC_JZ4760
#include <asm/mach-jz4760/jz4760lcdc.h>
#include <asm/mach-jz4760/jz4760ipu.h>
#include "jz4760_ipu.h"
#elif defined CONFIG_SOC_JZ4760B
#include <asm/mach-jz4760b/jz4760blcdc.h>
#include <asm/mach-jz4760b/jz4760bipu.h>
#include "jz4760_ipu.h"

#elif defined CONFIG_SOC_JZ4770
#include <asm/mach-jz4770/jz4770lcdc.h>
#include <asm/mach-jz4770/jz4770ipu.h>
#include "jz4760_ipu.h"
#endif

#include <asm/uaccess.h>

#ifndef PHYS
#define PHYS(n)  virt_to_phys(n)
#endif

#if defined(CONFIG_JZ_CIM)
extern unsigned int cim_read_dma();
#endif

img_param_t ipu_default_img = {};
struct YuvStride default_stride = {};

static int ipu_initialized = 0;
static unsigned int fb_w, fb_h;

static img_param_t *settings;
static struct Ration2m ratios[IPU_LUT_LEN * IPU_LUT_LEN];

extern unsigned char *lcd_frame0;
extern int get_lcd_width(void);
extern int get_lcd_height(void);

static void get_fbaddr_info(void) 
{
    fb_w = get_lcd_width();
    fb_h = get_lcd_height();
}

static void set_csc_config(int outW, int outH)
{
    unsigned int in_fmt = settings->ipu_data_fmt & IN_FMT_MASK;
    unsigned int out_fmt = settings->ipu_data_fmt & OUT_FMT_MASK;

    switch(out_fmt)
    {
        case OUT_FMT_RGB888:
            outW = outW << 2;
            break;
        case OUT_FMT_RGB555:
            outW = outW << 1;
            break;
        case OUT_FMT_RGB565:
            outW = outW << 1;
            break;
    }
    
    OUTREG32(IPU_IN_FM_GS, IN_FM_W(settings->in_width) | IN_FM_H(settings->in_height & ~0x1));
    OUTREG32(IPU_OUT_GS, OUT_FM_W(outW) | OUT_FM_H(outH));
    OUTREG32(IPU_OUT_STRIDE, settings->stride->out);
    OUTREG32(IPU_D_FMT, settings->ipu_data_fmt);

    // set CSC parameter
    if ((in_fmt != IN_FMT_YUV444) && (out_fmt != OUT_FMT_YUV422))
    {
        if (in_fmt != IN_FMT_PKG_RGB565)
        {
            settings->ipu_ctrl |= CSC_EN;

            OUTREG32(IPU_CSC_C0_COEF, YUV_CSC_C0);
            OUTREG32(IPU_CSC_C1_COEF, YUV_CSC_C1);
            OUTREG32(IPU_CSC_C2_COEF, YUV_CSC_C2);
            OUTREG32(IPU_CSC_C3_COEF, YUV_CSC_C3);
            OUTREG32(IPU_CSC_C4_COEF, YUV_CSC_C4);
            OUTREG32(IPU_CSC_OFSET_PARA, CHROM(YUV_CSC_CHROM) | LUMA(YUV_CSC_LUMA));
        }
    }
}

static int compute_ratios(void)
{
    int count = 0;
    int maxlength = IPU_LUT_LEN * IPU_LUT_LEN;
    int offset;
    int diff;
        
    int i;
    int j;

    // Populate the IPU Ratio table
    for(i = 1; i <= IPU_LUT_LEN; i++)
    {
        for(j = 1; j <= IPU_LUT_LEN; j++)
        {
            offset = (i - 1) * IPU_LUT_LEN + j - 1;
                    
            ratios[offset].ratio = i * IPU_RATIO_MUL / j;
            ratios[offset].n = i;
            ratios[offset].m = j;
        }
    }

    // Eliminate duplicate ratios to use minimum factorials
    for(i = 0; i < maxlength; i++) 
    {
        for (j = i + 1; j < maxlength; j++) 
        {
            diff = ratios[i].ratio - ratios[j].ratio;
                        
            if (diff > -100 && diff < 100) 
            {
                ratios[j].n = -1;
                ratios[j].m = -1;
            }
        }
    }

    // Collapse ratios to active entries
    for(i = 0; i < maxlength; i++) 
    {
        if(ratios[i].n != -1) 
        {
            if (count != i) ratios[count] = ratios[i];
                        
            count++;
        }
    }
    
    return count;
}

static int find_ratio(unsigned int ratio, unsigned int upscale, int ratio_count)
{
    int selected = ratio_count;
    int minimum = 32 * IPU_RATIO_MUL;
        
    int diff;
    int i;
        
    for(i = 0; i < ratio_count; i++) 
    {
        // If we're downscaling, skip upscaling options
        if(!upscale && (ratios[i].n & 1)) continue;

        if(ratio > ratios[i].ratio) diff = ratio - ratios[i].ratio;
        else diff = ratios[i].ratio - ratio;

        if(diff < minimum || i == 0) 
        {
            minimum = diff;
            selected = i;
        }
    }

    return selected;
}

static int set_ipu_buffer(void)
{
    unsigned int y_buffer;
    unsigned int u_buffer;
    unsigned int v_buffer;
    unsigned int tlb_y_buffer;
    unsigned int tlb_u_buffer;
    unsigned int tlb_v_buffer;
        
    PIPU_CTRL2 pipu_ctrl = (PIPU_CTRL2)(&settings->ipu_ctrl);

    y_buffer = ((unsigned int)settings->y_buffer);
    u_buffer = ((unsigned int)settings->u_buffer);
    v_buffer = ((unsigned int)settings->v_buffer);

    if(pipu_ctrl->spage_map != 0)
    {
        tlb_y_buffer = PHYS((void *)settings->tlb_y_buffer);
        tlb_u_buffer = PHYS((void *)settings->tlb_u_buffer);
        tlb_v_buffer = PHYS((void *)settings->tlb_v_buffer);

        y_buffer = tlb_y_buffer & 0xfff;
        u_buffer = tlb_u_buffer & 0xfff;
        v_buffer = tlb_v_buffer & 0xfff;

        // Input source TLB base address
        OUTREG32(IPU_Y_PHY_T_ADDR, tlb_y_buffer);
        OUTREG32(IPU_Y_PHY_T_ADDR, tlb_u_buffer);
        OUTREG32(IPU_Y_PHY_T_ADDR, tlb_v_buffer);
    } 
    else 
    {
        if((y_buffer == 0) || (u_buffer == 0) || (v_buffer == 0)) 
        {
            printk("IPU: Can not find buffer at (0x%x,0x%x,0x%x) physical address\n", (unsigned int)settings->y_buffer, (unsigned int)settings->u_buffer, (unsigned int)settings->v_buffer);
                        
            return -1;
        }
    }

    OUTREG32(IPU_Y_ADDR, y_buffer);
    OUTREG32(IPU_U_ADDR, u_buffer);
    OUTREG32(IPU_V_ADDR, v_buffer);

    return 0;
}

static void initialize_lookup_upscale(int srcN, int dstM, rsz_lut table[])
{
    int i, tick;
    unsigned int w_coef, factor1, factor2;
    
    for(i = 0, tick = 0; i < dstM; i++) 
    {
        factor1 = (i * srcN * IPU_RATIO_MUL) / dstM;
        factor2 = factor1 - (factor1 / IPU_RATIO_MUL * IPU_RATIO_MUL);
        w_coef = IPU_RATIO_MUL - factor2;

        table[i].coef = (unsigned int)(512 * w_coef / IPU_RATIO_MUL) & W_COEF0_MSK;
        table[i].out_n = 1;

        if(tick <= factor1) 
        {
            table[i].in_n = 1;
            tick += IPU_RATIO_MUL;
        } 
        else 
        {
            table[i].in_n = 0;
        }
    } 
}

static void initialize_lookup_downscale(int srcN, int dstM, rsz_lut table[])
{
    int i, tick, x;
    unsigned int w_coef, factor1, factor2;
    
    for (i = 0, tick = 0, x = 0; i < srcN; i++) 
    {
        factor1 = (tick * srcN + 1) * IPU_RATIO_MUL / dstM;
        table[i].in_n = 1;

        if(dstM == 1) 
        {
            table[i].coef = (i == 0) ? 256 : 0;
            table[i].out_n = (i == 0) ? 1  : 0;
        } 
        else 
        {
            if(((tick * srcN + 1) / dstM - i) >= 1) 
            {
                table[i].coef = 0;
            } 
            else 
            {
                if(factor1 - i * IPU_RATIO_MUL == 0) 
                {
                    table[i].coef = 512;
                } 
                else 
                {
                    factor2 = (tick * srcN ) / dstM * IPU_RATIO_MUL;
                    factor1 = factor1 - factor2;
                    w_coef = IPU_RATIO_MUL - factor1;
                    
                    table[i].coef = (unsigned int)(512 * w_coef / IPU_RATIO_MUL) & W_COEF0_MSK;
                }

                tick++;
            }

            if(((x * srcN + 1) / dstM - i) >= 1) 
            {
                table[i].out_n = 0;
            } 
            else 
            {
                table[i].out_n = 1;
                x++;
            }
        }
    }
}

static void initialize_lookup(int srcN, int dstM, int upscale, rsz_lut table[])
{
    if(upscale) initialize_lookup_upscale(srcN, dstM, table);
    else initialize_lookup_downscale(srcN, dstM, table);
}

static int compute_lookup(rsz_lut *lookup, int* oft_table, int* coef_table, int points)
{
    int length = 0;
    int oft_tmp = 0;
    int coef_tmp = 0;
    int i;
    
    for (i=0; i<points; i++) 
    {
        if(lookup[i].out_n)
        {
            oft_table[length] = (lookup[i].in_n == 0) ? 0 : oft_tmp;
            coef_table[length] = coef_tmp;
            
            coef_tmp = lookup[i].coef;
            oft_tmp = (lookup[i].in_n == 0) ? oft_tmp : lookup[i].in_n;
                        
            length++;
        } 
        else
        {
            oft_tmp = lookup[i].in_n + oft_tmp;
        }
    }

    if(lookup[0].out_n) 
    {
        oft_table[length] = (lookup[0].in_n == 0) ? 0 : oft_tmp;
        coef_table[length] = coef_tmp;
    }

    return length;
}

static void configure_resize(void)
{
    int selected_w, selected_h;
    int upscale_w, upscale_h;
    
    int height_points, width_points;
    int hcoef_count, vcoef_count;
    int i, ratio_count;
    
    rsz_lut horiz_lut[IPU_LUT_LEN];
    rsz_lut vert_lut[IPU_LUT_LEN];
    
    int hoft_table[IPU_LUT_LEN + 1];
    int voft_table[IPU_LUT_LEN + 1];
    
    int hcoef_table[IPU_LUT_LEN + 1];
    int vcoef_table[IPU_LUT_LEN + 1];

    unsigned int resizeWidth = (settings->out_width > fb_w) ? fb_w : settings->out_width;
    unsigned int resizeHeight = (settings->out_height > fb_h) ? fb_h : settings->out_height;

    printk("IPU: %dx%d -> %dx%d\n", settings->in_width, settings->in_height, settings->out_width, settings->out_height);

    // Zero out horizontal and vertical resize flags
    settings->ipu_ctrl &= ~(VRSZ_EN | HRSZ_EN);

    // No change in size
    if((settings->in_width == resizeWidth) && (settings->in_height == resizeHeight)) return;
    
    // Set IPU flags for which dimensions will be adjusted
    settings->ipu_ctrl |= (settings->in_width != resizeWidth) ? HRSZ_EN : 0;
    settings->ipu_ctrl |= (settings->in_height != resizeHeight) ? VRSZ_EN : 0;
        
    // Initialize the ipu table
    ratio_count = compute_ratios();

    // Determine if we're upscaling or downscaling the dimension
    upscale_w = (resizeWidth >= settings->in_width);
    upscale_h = (resizeHeight >= settings->in_height);

    // Get IPU table entry for resize ratio
    selected_w = find_ratio(settings->in_width * IPU_RATIO_MUL / resizeWidth, 1 /*upscale_w*/, ratio_count);
    selected_h = find_ratio(settings->in_height * IPU_RATIO_MUL / resizeHeight, 1 /*upscale_h*/, ratio_count);

    // Number of pixel points needed to correctly resize
    width_points = upscale_w ? ratios[selected_w].m : ratios[selected_w].n;
    height_points = upscale_h ? ratios[selected_h].m : ratios[selected_h].n;

    initialize_lookup(ratios[selected_w].n, ratios[selected_w].m, upscale_w, horiz_lut);
    initialize_lookup(ratios[selected_h].n, ratios[selected_h].m, upscale_h, vert_lut);

    hcoef_count = compute_lookup(horiz_lut, hoft_table, hcoef_table, width_points);
    vcoef_count = compute_lookup(vert_lut, voft_table, vcoef_table, height_points);
    
    OUTREG32(IPU_RSZ_COEF_INDEX, ((vcoef_count - 1) << VE_IDX_H_BIT) | ((hcoef_count- 1)  << HE_IDX_W_BIT));
    
    // Program width pixel points
    SETREG32(IPU_VRSZ_LUT_BASE, 1 << V_CONF_BIT);

    for(i = 0; i < vcoef_count; i++) 
    {
        OUTREG32(IPU_VRSZ_LUT_BASE, ((vcoef_table[i+1] & W_COEF0_MSK) << W_COEF0_BIT) | ((voft_table[i+1] & V_OFT_MSK) << V_OFT_BIT));
    }
    
    // Program height pixel points
    SETREG32(IPU_HRSZ_LUT_BASE, 1 << H_CONF_BIT);
        
    for(i = 0; i < hcoef_count; i++) 
    {
        OUTREG32(IPU_HRSZ_LUT_BASE, ((hcoef_table[i+1] & W_COEF0_MSK) << W_COEF0_BIT) | ((hoft_table[i+1] & H_OFT_MSK) << H_OFT_BIT));
    }
}

void ipu_init(img_param_t *pimg)
{
    int in_fmt;
    int out_fmt;
    
    unsigned int *y_stride, *u_stride, *v_stride, *out_stride;

    if(!pimg)
    {
        printk("IPU: Invalid settings passed\n");
        return;
    }
        
    get_fbaddr_info();
        
    settings = pimg;
    in_fmt = settings->ipu_data_fmt & IN_FMT_MASK;
    out_fmt = settings->ipu_data_fmt & OUT_FMT_MASK;

    y_stride = &(settings->stride->y);
    u_stride = &(settings->stride->u);
    v_stride = &(settings->stride->v);
    out_stride = &(settings->stride->out);

    if(!ipu_initialized) 
    {
        CLRREG32(CPM_CLKGR0, CLKGR0_IPU); // run ipu clock
        CLRREG32(IPU_CTRL, IPU_RUN);
        SETREG32(IPU_CTRL, CHIP_EN);
        SETREG32(IPU_CTRL, IPU_RST); // reset controller
        CLRREG32(IPU_CTRL, IPU_RST);
                
        while((INREG32(IPU_STATUS) & OUT_END) == 0) 
        {
            printk("IPU: Time out\n"); // wait the end flag
        }
    } 
    else 
    {
        while((INREG32(IPU_CTRL) & IPU_RUN) && ((INREG32(IPU_STATUS) & OUT_END) == 0)); // wait the end flag
                
        SETREG32(IPU_CTRL, IPU_RST); // reset controller
        CLRREG32(IPU_CTRL, IPU_RST);
                
        while((INREG32(IPU_STATUS) & OUT_END) == 0); // wait the end flag
    }
        
    if((in_fmt == IN_FMT_YUV444) && (out_fmt != OUT_FMT_YUV422)) 
    {
        settings->ipu_ctrl &= ~SPKG_SEL; //Separated Frame
    }

    configure_resize();
    set_csc_config(settings->out_width, settings->out_height);

    if(set_ipu_buffer() < 0) return;

    OUTREG32(IPU_Y_STRIDE, *y_stride);
    OUTREG32(IPU_UV_STRIDE, U_STRIDE(*u_stride) | V_STRIDE(*v_stride));
    OUTREG32(IPU_CTRL, settings->ipu_ctrl | CHIP_EN);

    ipu_initialized = 1;
}


void ipu_start(int src_w, int src_h, int dst_w, int dst_h)
{
    if(ipu_initialized) return;

    settings = &ipu_default_img;
    settings->in_width = src_w;
    settings->in_height = src_h;
    settings->in_bpp = 16;
    settings->out_width = dst_w;
    settings->out_height = dst_h;

    settings->ipu_ctrl = LCDC_SEL | SPKG_SEL | FM_IRQ_EN;
    settings->ipu_data_fmt = IN_FMT_PKG_RGB565 | OUT_FMT_RGB565;

    switch(settings->in_bpp) 
    {
        case 32:
            settings->ipu_data_fmt |= IN_FMT_YUV444; // 2 << IN_FMT_BIT;
            break;
        case 16:
        default:
            settings->ipu_data_fmt |= IN_FMT_YUV422; // 3 << IN_FMT_BIT;
            break;
    }

    settings->y_buffer = (unsigned char*)PHYS((void *)lcd_frame0);
    settings->u_buffer = (unsigned char*)PHYS((void *)lcd_frame0);
    settings->v_buffer = (unsigned char*)PHYS((void *)lcd_frame0);
    settings->stride = &default_stride;
    settings->stride->out = dst_w*2;
    settings->stride->y = src_w*2;
    settings->stride->u = src_w/2;
    settings->stride->v = src_w/2;

    ipu_init(settings);

    while(((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0);

    // ipu start
    REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
    REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
}

void ipu_stop(void)
{
    if(!ipu_initialized) return;
        
    ipu_initialized = 0;
        
    while(((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0);
        
    REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
    REG32(IPU_V_BASE + REG_CTRL) &= ~IPU_RUN;
}

extern unsigned int frame_yoffset;
void ipu_update_address(void)
{
    REG32(IPU_V_BASE + REG_Y_ADDR) = (unsigned int)PHYS(lcd_frame0 + frame_yoffset);
}

EXPORT_SYMBOL(ipu_update_address);
EXPORT_SYMBOL(ipu_stop);
EXPORT_SYMBOL(ipu_start);

