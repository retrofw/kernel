/*
 * Linux/sound/oss/jzoss/jz4770/dlv.c
 *
 * DLV CODEC driver for Ingenic Jz4770 MIPS processor
 *
 * 2011-11-xx	liulu <lliu@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/vmalloc.h>
#include <asm/hardirq.h>
#include <asm/jzsoc.h>
#include <linux/fs.h>

#include "../../jz_snd.h"
#include "dlv.h"

#define TURN_ON_SB_HP_WHEN_INIT  1

static struct semaphore *g_dlv_sem = 0;
static struct work_struct dlv_irq_work;
static spinlock_t dlv_irq_lock;
static int g_replay_volume = 0;
static int g_is_replaying = 0;
static int g_is_recording = 0;

//**************************************************************************
// debug
//**************************************************************************

static int jz4770_dlv_debug = 0;
module_param(jz4770_dlv_debug, int, 0644);
#define JZ4770_DLV_DEBUG_MSG(msg...)			\
	do {					\
		if (jz4770_dlv_debug)		\
			printk("jz4770_dlv: " msg);	\
	} while(0)

//**************************************************************************
// CODEC registers access routines 
//**************************************************************************

/**
 * CODEC read register
 *
 * addr:	address of register
 * return:	value of register
 */
static inline int dlv_read_reg(int addr)
{
	volatile int reg;

	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	__icdc_set_addr(addr);
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	reg = __icdc_get_value();
	return __icdc_get_value();
}

/**
 * CODEC write register
 *
 * addr:	address of register
 * val:		value to set
 */
void dlv_write_reg(int addr, int val)
{
	volatile int reg;

	while (__icdc_rgwr_ready()) {
        printk("1. __icdc_rgwr_ready()=%d\n" ,__icdc_rgwr_ready());
		;//nothing...
	}
	REG_ICDC_RGADW = ICDC_RGADW_RGWR | ((addr << ICDC_RGADW_RGADDR_LSB) | val);
	//__icdc_set_rgwr();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	reg = __icdc_rgwr_ready();
	while (__icdc_rgwr_ready()) {
        printk("2. __icdc_rgwr_ready()=%d\n" ,__icdc_rgwr_ready());
		;//nothing...
	}
}

static inline void dlv_sleep_wait_bitset(int reg, unsigned bit_mask, int stime, int line)
{
	while(!(dlv_read_reg(reg) & bit_mask)) {
		msleep(stime);
	}
}

static inline void dlv_sleep_wait_bitclear(int reg, unsigned bit_mask, int stime)
{
	while((dlv_read_reg(reg) & bit_mask))
		msleep(stime);
}

/**
 * CODEC write a bit of a register
 *
 * addr:	address of register
 * bitval:	bit value to modifiy
 * mask_bit:	indicate which bit will be modifiy
 */
static int dlv_write_reg_bit(int addr, int bitval, int bit_mask)
{
	int val = dlv_read_reg(addr);

	if (bitval)
		val |= bit_mask;
	else
		val &= ~bit_mask;
	dlv_write_reg(addr, val);

	return 1;
}

static int dlv_update_reg(int addr,  int val, int mask) {
	int old_val = dlv_read_reg(addr);

	old_val &= ~mask;
	old_val |= (val & mask);

	dlv_write_reg(addr, old_val);

	return 1;
}

//**************************************************************************
//   DLV CODEC operations routines
//**************************************************************************
static inline void turn_on_dac(int timeout)
{
	if(__dlv_get_dac_mute()){
		/* clear IFR_GUP */
		__dlv_set_irq_flag(DLC_IFR_GUP);
		mdelay(300);
		/* turn on dac */
		__dlv_disable_dac_mute();
		/* wait IFR_GUP set */
		dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_GUP, timeout,__LINE__);
		udelay(100);  /* 4 SYS_CLK cycles */
		__dlv_set_irq_flag(DLC_IFR_GUP);
	}
}

static inline void turn_off_dac(int timeout)
{
 	if (!(__dlv_get_dac_mute())){
		/* clear IFR_GDO */
		__dlv_set_irq_flag(DLC_IFR_GDO);
		/* turn off dac */
		__dlv_enable_dac_mute();
		/* wait IFR_GDO set */
		dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_GDO, timeout,__LINE__);
		udelay(100);
		__dlv_set_irq_flag(DLC_IFR_GDO);
	}
}

static inline void turn_on_sb_hp(void)
{
	if (__dlv_get_sb_hp() != POWER_ON){
		/* clear IFR_RUP */
		__dlv_set_irq_flag(DLC_IFR_RUP);
		/* turn on sb_hp */
		__dlv_set_cap_less();
        __dlv_switch_sb_hp(POWER_ON);
		/* wait IFR_RUP set */
        dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_RUP, 100,__LINE__);
		udelay(100);
		__dlv_set_irq_flag(DLC_IFR_RUP);
	}

}

static inline void turn_off_sb_hp(void)
{
	if (__dlv_get_sb_hp() != POWER_OFF){
		/* clear IFR_RDO */
		__dlv_set_irq_flag(DLC_IFR_RDO);
		/* turn off sb_hp */
		__dlv_set_cap_couple();
        	__dlv_switch_sb_hp(POWER_OFF);
		/* wait IFR_RDO set */
        	dlv_sleep_wait_bitset(DLC_IFR, DLC_IFR_RDO, 100,__LINE__);
		udelay(100);
		__dlv_set_irq_flag(DLC_IFR_RUP);
	}
}

//************************************************************************
//  dlv control
//************************************************************************

static void dlv_shutdown(void) {
	turn_off_dac(5);

    __aic_write_tfifo(0x0);
    __aic_write_tfifo(0x0);
	__i2s_enable_replay();
	msleep(1);

	turn_off_sb_hp();
	mdelay(1);
	__dlv_enable_hp_mute();
 	mdelay(1);
	__dlv_switch_sb_dac(POWER_OFF);
	mdelay(1);
	__dlv_switch_sb_sleep(POWER_OFF);
	mdelay(1);
	__dlv_switch_sb(POWER_OFF);
	mdelay(1);
}

static void dlv_startup(void){
    __dlv_switch_sb(POWER_ON);
	mdelay(300);

    __dlv_switch_sb_sleep(POWER_ON);
	mdelay(400);

#if TURN_ON_SB_HP_WHEN_INIT == 1
    __dlv_switch_sb_dac(POWER_ON);
    mdelay(1);
    turn_on_sb_hp();
#endif
}

//**************************************************************************
// CODEC mode ctrl
//**************************************************************************

static unsigned int g_current_mode;
static unsigned int g_replay_wc;
static unsigned int g_record_wc;
static unsigned int g_bypass_wc;
static unsigned int g_jack_on;

static struct dlv_codec_mode g_codec_modes[] = {
    //adc
    {CODEC_MODE_MIC1_2_ADC, CODEC_MODE_WC_MIC1_V | CODEC_MODE_WC_ADC_V,
     CODEC_MODE_CD_GEN(3, ADC_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(0, ADC_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC2_2_ADC, CODEC_MODE_WC_MIC2_V | CODEC_MODE_WC_ADC_V,
     CODEC_MODE_CD_GEN(3, ADC_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(1, ADC_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC1_MIC2_2_ADC, CODEC_MODE_WC_MIC1_V | CODEC_MODE_WC_MIC2_V | CODEC_MODE_WC_ADC_V,
     CODEC_MODE_CD_GEN(3, ADC_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(0, ADC_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE)
    },
    {CODEC_MODE_MIC2_MIC1_2_ADC, CODEC_MODE_WC_MIC1_V | CODEC_MODE_WC_MIC2_V | CODEC_MODE_WC_ADC_V,
     CODEC_MODE_CD_GEN(3, ADC_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(1, ADC_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE)
    },
    {CODEC_MODE_LINE_IN_2_ADC,  CODEC_MODE_WC_LINE_IN_V | CODEC_MODE_WC_ADC_V,
     CODEC_MODE_CD_GEN(3, ADC_SEL), CODEC_MODE_CD_GEN(2, ADC_SEL)
    },
    //hp
    {CODEC_MODE_MIC1_2_HP,  CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_HP,
     CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(0, HP_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC2_2_HP,  CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_HP,
     CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(1, HP_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC1_MIC2_2_HP,   CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_HP,
     CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(0, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE)
    },
    {CODEC_MODE_MIC2_MIC1_2_HP,   CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_HP,
     CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(1, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE)
    },
    {CODEC_MODE_LINE_IN_2_HP,  CODEC_MODE_WC_LINE_IN | CODEC_MODE_WC_HP,
     CODEC_MODE_CD_GEN(3, HP_SEL) , CODEC_MODE_CD_GEN(2, HP_SEL)
    },
    {CODEC_MODE_DAC_2_HP,  CODEC_MODE_WC_DAC_V | CODEC_MODE_WC_HP_V,
     CODEC_MODE_CD_GEN(3, HP_SEL) , CODEC_MODE_CD_GEN(3, HP_SEL)
    },
    //line out
    {CODEC_MODE_MIC1_2_LO, CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_LINE_OUT,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(0, LO_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC2_2_LO, CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_LINE_OUT,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(1, LO_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC1_MIC2_2_LO, CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_LINE_OUT,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), CODEC_MODE_CD_GEN(0, LO_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE)
    },
    {CODEC_MODE_LINE_IN_2_LO, CODEC_MODE_WC_LINE_IN | CODEC_MODE_WC_LINE_OUT,
     CODEC_MODE_CD_GEN(3, LO_SEL), CODEC_MODE_CD_GEN(2, LO_SEL)
    },
    {CODEC_MODE_DAC_2_LO, CODEC_MODE_WC_DAC_V | CODEC_MODE_WC_LINE_OUT_V,
     CODEC_MODE_CD_GEN(3, LO_SEL), CODEC_MODE_CD_GEN(3, LO_SEL)
    },
    //jack detect
    {CODEC_MODE_MIC1_2_JD,  CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_JD,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE),
     CODEC_MODE_CD_GEN(0, LO_SEL) | CODEC_MODE_CD_GEN(0, HP_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC2_2_JD,  CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_JD,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), 
     CODEC_MODE_CD_GEN(1, LO_SEL) | CODEC_MODE_CD_GEN(1, HP_SEL) | CODEC_MODE_CD_GEN(0, MIC_STE)
    },
    {CODEC_MODE_MIC1_MIC2_2_JD,   CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_JD,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), 
     CODEC_MODE_CD_GEN(0, LO_SEL) | CODEC_MODE_CD_GEN(0, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE)
    },
    {CODEC_MODE_MIC2_MIC1_2_JD,   CODEC_MODE_WC_MIC2 | CODEC_MODE_WC_MIC1 | CODEC_MODE_WC_JD,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(3, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE), 
     CODEC_MODE_CD_GEN(1, LO_SEL) | CODEC_MODE_CD_GEN(1, HP_SEL) | CODEC_MODE_CD_GEN(1, MIC_STE)
    },
    {CODEC_MODE_LINE_IN_2_JD,  CODEC_MODE_WC_LINE_IN | CODEC_MODE_WC_JD,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(3, HP_SEL) , 
     CODEC_MODE_CD_GEN(2, LO_SEL) | CODEC_MODE_CD_GEN(2, HP_SEL)
    },
    {CODEC_MODE_DAC_2_JD,  CODEC_MODE_WC_DAC_V | CODEC_MODE_WC_JD_V,
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(3, HP_SEL) , 
     CODEC_MODE_CD_GEN(3, LO_SEL) | CODEC_MODE_CD_GEN(3, HP_SEL)
    },
};

static void dlv_mode_set_route(unsigned int cd_mask, unsigned int cd_val)
{
    int value;

    if(cd_mask & CODEC_MODE_CD_GEN(3, ADC_SEL)){  
        //setting ADC_SEL
        value = CODEC_MODE_CD_GET_VALUE(cd_val, ADC_SEL, 3);
        __dlv_adc_set_route(value);
    }

    if(cd_mask & CODEC_MODE_CD_GEN(3, HP_SEL)){  
        //setting HP_SEL
        value = CODEC_MODE_CD_GET_VALUE(cd_val, HP_SEL, 3);
        __dlv_hp_set_route(value);
    }

    if(cd_mask & CODEC_MODE_CD_GEN(3, LO_SEL)){  
        //setting ADC_SEL
        value = CODEC_MODE_CD_GET_VALUE(cd_val, LO_SEL, 3);
        __dlv_lo_set_route(value);
    }

    if(cd_mask & CODEC_MODE_CD_GEN(3, MIC_STE)){  
        //setting MIC_STEREO
        value = CODEC_MODE_CD_GET_VALUE(cd_val, MIC_STE, 3);
        if(value){
            __dlv_enable_mic_stereo();
        }else{
            __dlv_disable_mic_stereo();
        }
    }
}

static void dlv_mode_set_widget(unsigned int wc){
    g_bypass_wc = 0;
    g_replay_wc = 0;
    g_record_wc = 0;
    if(wc & CODEC_MODE_WC_MIC1){
        __dlv_enable_mic1();
        g_bypass_wc |= CODEC_MODE_WC_MIC1;
    }else{
        __dlv_disable_mic1();
    }

    if(wc & CODEC_MODE_WC_MIC2){
        __dlv_enable_mic2();
        g_bypass_wc |= CODEC_MODE_WC_MIC2;
    }else{
        __dlv_disable_mic2();
    }

    if((wc & CODEC_MODE_WC_MIC2) || (wc & CODEC_MODE_WC_MIC1)){
        __dlv_switch_sb_micbias(POWER_ON);
    }else{
        __dlv_switch_sb_micbias(POWER_OFF);
    }
    
    if(wc & CODEC_MODE_WC_LINE_IN){
        __dlv_enable_li_for_bypass();
        g_bypass_wc |= CODEC_MODE_WC_LINE_IN;
    }else{
        __dlv_disable_li_for_bypass();
    }    

    if(wc & CODEC_MODE_WC_HP){
#if TURN_ON_SB_HP_WHEN_INIT == 0
        turn_on_sb_hp();
#endif
        __dlv_disable_hp_mute();
        g_bypass_wc |= CODEC_MODE_WC_HP;
    }else{
        __dlv_enable_hp_mute();
#if TURN_ON_SB_HP_WHEN_INIT == 0
        turn_off_sb_hp();
#endif
    }    

    if(wc & CODEC_MODE_WC_LINE_OUT){
        __dlv_switch_sb_line_out(POWER_ON);
        __dlv_disable_lineout_mute();
        g_bypass_wc |= CODEC_MODE_WC_LINE_OUT;
    }else{
        __dlv_switch_sb_line_out(POWER_OFF);
        __dlv_enable_lineout_mute();
    }    

    if(wc & CODEC_MODE_WC_JD){
        if(g_jack_on){
#if TURN_ON_SB_HP_WHEN_INIT == 0
            turn_on_sb_hp();
#endif
            __dlv_disable_hp_mute();
        }else{
            __dlv_switch_sb_line_out(POWER_ON);
            __dlv_disable_lineout_mute();
        }
        g_bypass_wc |= CODEC_MODE_WC_JD;
    }else{
        if(g_jack_on){
            __dlv_enable_hp_mute();
#if TURN_ON_SB_HP_WHEN_INIT == 0
            turn_off_sb_hp();
#endif
        }else{
            __dlv_switch_sb_line_out(POWER_OFF);
            __dlv_enable_lineout_mute();
        }
    }    

    if(wc & CODEC_MODE_WC_ADC_V){
        g_record_wc |= CODEC_MODE_WC_ADC_V;
    }
 
    if(wc & CODEC_MODE_WC_DAC_V){
        g_replay_wc |= CODEC_MODE_WC_DAC_V;
    }

    if(wc & CODEC_MODE_WC_MIC1_V){
        g_record_wc |= CODEC_MODE_WC_MIC1_V;
    }    

    if(wc & CODEC_MODE_WC_MIC2_V){
        g_record_wc |= CODEC_MODE_WC_MIC2_V;
    }    

    if(wc & CODEC_MODE_WC_LINE_IN_V){
        g_record_wc |= CODEC_MODE_WC_LINE_IN_V;
    }    

    if(wc & CODEC_MODE_WC_HP_V){
        g_replay_wc |= CODEC_MODE_WC_HP_V;
    }    

    if(wc & CODEC_MODE_WC_LINE_OUT_V){
        g_replay_wc |= CODEC_MODE_WC_LINE_OUT_V;
    }    

    if(wc & CODEC_MODE_WC_JD_V){
        g_replay_wc |= CODEC_MODE_WC_JD_V;
    }    
}

static int dlv_mode_ctrl(unsigned int mode)
{
    int i;
    unsigned int cd_mask, cd_mask_l, cd_val, wc, mode_l, mode_bit;
    
	JZ4770_DLV_DEBUG_MSG("enter %s, mode = 0x%x\n", __func__, mode);
    //1. mode collision detection
    if(g_is_replaying || g_is_recording){
        return -1;
    }
    cd_mask = cd_mask_l = cd_val = wc = 0;
    mode_l = mode;
    while(mode_l != 0){ // detecte every mode 
        mode_bit = mode_l & (mode_l - 1);
        mode_bit ^= mode_l;
        mode_l = mode_l & (mode_l - 1);

        for(i = 0; i < sizeof(g_codec_modes)/sizeof(struct dlv_codec_mode); ++i){
            if(mode_bit == g_codec_modes[i].id){ 
                cd_mask_l = g_codec_modes[i].cd_mask & cd_mask;
                if(cd_mask_l){
                    if(((cd_mask_l & cd_val) & cd_val) != (cd_mask_l & g_codec_modes[i].cd_value)){
                        //catch it
                        return -1;
                    }
                }
                cd_mask |= g_codec_modes[i].cd_mask;
                cd_val |= g_codec_modes[i].cd_value;
                wc |= g_codec_modes[i].widget_ctrl;
                break;
            }
        }
    }

	JZ4770_DLV_DEBUG_MSG("enter %s, cd_mask = 0x%x\n", __func__, cd_mask);
	JZ4770_DLV_DEBUG_MSG("enter %s, cd_val = 0x%x\n", __func__, cd_val);
	JZ4770_DLV_DEBUG_MSG("enter %s, wc = 0x%x\n", __func__, wc);

    //2. setting route
    dlv_mode_set_route(cd_mask, cd_val);

    //3. setting widget
    dlv_mode_set_widget(wc);

    g_current_mode = mode;
    return 0;
}

//**************************************************************************
// jack detect handle
//**************************************************************************

static void dlv_jack_ctrl(int on)
{
    JZ4770_DLV_DEBUG_MSG("enter %s, on = %d\n", __func__, on);
    if((g_bypass_wc & CODEC_MODE_WC_JD) || //bypass
       ((g_replay_wc & CODEC_MODE_WC_JD_V) && g_is_replaying)){//replay
        if(on){
            JZ4770_DLV_DEBUG_MSG("enter %s, jack on\n", __func__);
            //open hp
#if TURN_ON_SB_HP_WHEN_INIT == 0
            turn_on_sb_hp();
#endif
            __dlv_disable_hp_mute();
            //close line out
            __dlv_switch_sb_line_out(POWER_OFF);
            __dlv_enable_lineout_mute();
        }else{
            JZ4770_DLV_DEBUG_MSG("enter %s, jack off\n", __func__);
            //open line out
            __dlv_switch_sb_line_out(POWER_ON);
            __dlv_disable_lineout_mute();
            //close hp
            __dlv_enable_hp_mute();
#if TURN_ON_SB_HP_WHEN_INIT == 0
            turn_off_sb_hp();
#endif
        }
    }
    g_jack_on = on;
}

//**************************************************************************
//  irq handle
//**************************************************************************

/**
 * CODEC short circut handler
 *
 * To protect CODEC, CODEC will be shutdown when short circut occured.
 * Then we have to restart it.
 */
static inline void dlv_short_circut_handler(void)
{
	unsigned short curr_vol;
	unsigned int	dlv_ifr, delay;

#define VOL_DELAY_BASE 22               //per VOL delay time in ms

	printk("JZ DLV: Short circut detected! restart CODEC hp out finish.\n");

	curr_vol = dlv_read_reg(DLC_GCR_HPL);
	delay = VOL_DELAY_BASE * (0x20 - (curr_vol & 0x1f));

	/* min volume */
	__dlv_set_hp_volume(0x1f);

	printk("Short circut volume delay %d ms curr_vol=%x \n", delay,curr_vol);
	msleep(delay);

    dlv_shutdown();

	while (1) {
		dlv_ifr = __dlv_get_irq_flag();
		printk("waiting for short circuit recover finish ----- dlv_ifr = 0x%02x\n", dlv_ifr);
		if ((dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2)) == 0)
			break;
		__dlv_set_irq_flag((DLC_IFR_SCLR | DLC_IFR_SCMC2));
		msleep(10);
	}

    dlv_startup();

    if(g_replay_wc & CODEC_MODE_WC_HP){
#if TURN_ON_SB_HP_WHEN_INIT == 0
        turn_on_sb_hp();
#endif
        __dlv_disable_hp_mute();
    }

    if((g_replay_wc & CODEC_MODE_WC_JD) && g_jack_on){
#if TURN_ON_SB_HP_WHEN_INIT == 0
            turn_on_sb_hp();
#endif
            __dlv_disable_hp_mute();
    }

    if(g_is_replaying){
#if TURN_ON_SB_HP_WHEN_INIT == 0
        __dlv_switch_sb_dac(POWER_ON);
        mdelay(1);
        if(((g_replay_wc & CODEC_MODE_WC_HP_V) && !(g_bypass_wc & CODEC_MODE_WC_HP)) || 
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && g_jack_on)
           ){
            turn_on_sb_hp();
        }
#endif
        if(g_replay_volume > 0){
            turn_on_dac(5);
        }
        if(((g_replay_wc & CODEC_MODE_WC_HP_V) && !(g_bypass_wc & CODEC_MODE_WC_HP)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && g_jack_on)
           ){
            __dlv_disable_hp_mute();
        }
    }

	__dlv_set_hp_volume(curr_vol);
	msleep(delay);
}

/**
 * CODEC work queue handler
 *
 * Handle bottom-half of SCLR & JACKE irq
 *
 */
static unsigned int s_dlv_sr_jack = 2;  //init it to a invalid value
static void dlv_irq_work_handler(struct work_struct *work)
{
	unsigned int	dlv_ifr, dlv_sr_jack;

	DLV_LOCK();

    dlv_ifr = __dlv_get_irq_flag();
    printk("JZ DLV: irq detected! dlv_ifr = 0x%02x\n",dlv_ifr);

    //short circut handle
    while(dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2)){
        dlv_short_circut_handler();
		dlv_ifr = __dlv_get_irq_flag();
		/* Updata SCLR */
		__dlv_set_irq_flag(DLC_IFR_SCLR | DLC_IFR_SCMC2);
    }

    //detect jack
    if(dlv_ifr & DLC_IFR_JACK){
        msleep(200);
        dlv_sr_jack = __dlv_get_sr_jack();
        if(dlv_sr_jack != s_dlv_sr_jack){
            if(dlv_sr_jack){
                dlv_jack_ctrl(1);
            }else{
                dlv_jack_ctrl(0);
            }
        }
        s_dlv_sr_jack = __dlv_get_sr_jack();
        __dlv_set_irq_flag(DLC_IFR_JACK);
    }

    /* Unmask*/
    __dlv_set_irq_mask(ICR_COMMON_MASK);
	DLV_UNLOCK();
}

/**
 * IRQ routine
 *
 * Now we are only interested in SCLR.
 */
static irqreturn_t dlv_codec_irq(int irq, void *dev_id)
{
	unsigned char dlv_ifr;
	unsigned long flags;

	printk("===>enter dlv_codec_irq!!!\n");
	spin_lock_irqsave(&dlv_irq_lock, flags);
	dlv_ifr = __dlv_get_irq_flag();
	__dlv_set_irq_mask(ICR_ALL_MASK);

	if (!(dlv_ifr & (DLC_IFR_SCLR | DLC_IFR_SCMC2 | DLC_IFR_JACK))) {
		/* CODEC may generate irq with flag = 0xc0.
		 * We have to ingore it in this case as there is no mask for the reserve bit.
		 */
		printk("AIC interrupt, IFR = 0x%02x\n", dlv_ifr);
		__dlv_set_irq_mask(ICR_COMMON_MASK);
		spin_unlock_irqrestore(&dlv_irq_lock, flags);
		return IRQ_HANDLED;
	} else {
		spin_unlock_irqrestore(&dlv_irq_lock, flags);
		/* Handle SCLR and JACK in work queue. */
		schedule_work(&dlv_irq_work);
		return IRQ_HANDLED;
	}
}

//**************************************************************************
//  codec triggle handle
//**************************************************************************
static int dlv_set_replay_volume(int vol);
static int __init dlv_event_init(void)
{
	cpm_start_clock(CGM_AIC);

	/* init codec params */
	/* ADC/DAC: serial + i2s */
    __dlv_set_dac_serial();
    __dlv_dac_i2s_mode();
    __dlv_set_adc_serial();
    __dlv_adc_i2s_mode();

	/* The generated IRQ is a high level */
	__dlv_set_int_form(DLC_ICR_INT_HIGH);
	__dlv_set_irq_mask(ICR_COMMON_MASK);
    __dlv_set_irq_flag(IFR_ALL_FLAG);

	/* 12M */
	__dlv_set_12m_crystal();

	//__dlv_set_10kohm_load(); //1uF
	__dlv_set_16ohm_load(); // 220uF

	/* disable AGC */
    __dlv_disable_agc();

	/* default to MICDIFF */
    __dlv_enable_micdiff();

	/* mute lineout/HP */
    __dlv_enable_hp_mute();
    __dlv_enable_lineout_mute();

	/* DAC lrswap */
    __dlv_set_dac_lrswap();

	/* ADC lrswap */
    __dlv_enable_adc_lrswap();

	/* default to cap-less mode(0) */
    __dlv_set_cap_less();

    dlv_startup();

    //default enable adc left only
	__dlv_enable_adc_left_only();

    //default disable dac left only
	__dlv_disable_dac_left_only();

    //set default mode
    dlv_mode_ctrl(CODEC_MODE_MIC1_2_ADC | CODEC_MODE_DAC_2_HP);

    return 0;
}

static int __exit dlv_event_deinit(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);
    dlv_shutdown();
    return 0;
}

static int dlv_event_replay_start(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);
    //in order
    if(g_replay_wc & CODEC_MODE_WC_DAC_V){
#if TURN_ON_SB_HP_WHEN_INIT == 0
        __dlv_switch_sb_dac(POWER_ON);
        mdelay(1);
        if(((g_replay_wc & CODEC_MODE_WC_HP_V) && !(g_bypass_wc & CODEC_MODE_WC_HP)) || 
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && g_jack_on)
           ){
            turn_on_sb_hp();
        }
#endif
        if(((g_replay_wc & CODEC_MODE_WC_LINE_OUT_V) && !(g_bypass_wc & CODEC_MODE_WC_LINE_OUT)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && !(g_jack_on))           
           ){
            __dlv_switch_sb_line_out(POWER_ON);
        }

        if(g_replay_volume > 0){
            turn_on_dac(5);
        }

        if(((g_replay_wc & CODEC_MODE_WC_HP_V) && !(g_bypass_wc & CODEC_MODE_WC_HP)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && g_jack_on)
           ){
            __dlv_disable_hp_mute();
        }

        if(((g_replay_wc & CODEC_MODE_WC_LINE_OUT_V) && !(g_bypass_wc & CODEC_MODE_WC_LINE_OUT)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && !(g_jack_on))           
           ){
            __dlv_disable_lineout_mute();
        }
    }
    
    g_is_replaying = 1;
    return 0;
}

/* static int first_from_init_i2s = 1; */
static int dlv_event_replay_stop(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);

    if(g_replay_wc & CODEC_MODE_WC_DAC_V){
        turn_off_dac(5);
        if(((g_replay_wc & CODEC_MODE_WC_HP_V) && !(g_bypass_wc & CODEC_MODE_WC_HP)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && g_jack_on)
           ){
            __dlv_enable_hp_mute();
        }
        if(((g_replay_wc & CODEC_MODE_WC_LINE_OUT_V) && !(g_bypass_wc & CODEC_MODE_WC_LINE_OUT)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && !(g_jack_on))           
           ){
            __dlv_enable_lineout_mute();
        }
        /* anti-pop workaround */
        __aic_write_tfifo(0x0);
        __aic_write_tfifo(0x0);
        __i2s_enable_replay();
        __i2s_enable();
        mdelay(1);
        __i2s_disable_replay();
        __i2s_disable();

#if TURN_ON_SB_HP_WHEN_INIT == 0
        if(((g_replay_wc & CODEC_MODE_WC_HP_V) && !(g_bypass_wc & CODEC_MODE_WC_HP)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && g_jack_on)
           ){
            turn_off_sb_hp();
        }
        __dlv_switch_sb_dac(POWER_OFF);
#endif
        if(((g_replay_wc & CODEC_MODE_WC_LINE_OUT_V) && !(g_bypass_wc & CODEC_MODE_WC_LINE_OUT)) ||
           (((g_replay_wc & CODEC_MODE_WC_JD_V) && !(g_bypass_wc & CODEC_MODE_WC_JD)) && !(g_jack_on))
           ){
            __dlv_switch_sb_line_out(POWER_OFF);
        }
    }
    g_is_replaying = 0;
    mdelay(10);
    return 0;
}

static int dlv_event_record_start(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);

    if(g_record_wc & CODEC_MODE_WC_ADC_V){
        __dlv_enable_adc();
        mdelay(20);
        if((g_record_wc & CODEC_MODE_WC_MIC1_V) && !(g_bypass_wc & CODEC_MODE_WC_MIC1)){
            __dlv_switch_sb_micbias(POWER_ON);
            __dlv_enable_mic1();
        }
        if((g_record_wc & CODEC_MODE_WC_MIC2_V) && !(g_bypass_wc & CODEC_MODE_WC_MIC2)){
            __dlv_switch_sb_micbias(POWER_ON);
            __dlv_enable_mic2();
        }
        if((g_record_wc & CODEC_MODE_WC_LINE_IN_V)){
            __dlv_enable_li_for_adc();
        }
    }
    g_is_recording = 1;
    return 0;
}

static int dlv_event_record_stop(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);

    if(g_record_wc & CODEC_MODE_WC_ADC_V){
        __dlv_disable_adc();
        mdelay(2);
        if((g_record_wc & CODEC_MODE_WC_MIC1_V) && !(g_bypass_wc & CODEC_MODE_WC_MIC1)){
            __dlv_disable_mic1();
        }
        if((g_record_wc & CODEC_MODE_WC_MIC2_V) && !(g_bypass_wc & CODEC_MODE_WC_MIC2)){
            __dlv_disable_mic2();
        }
        if(!(g_bypass_wc & CODEC_MODE_WC_MIC2) && !(g_bypass_wc & CODEC_MODE_WC_MIC1)){
            __dlv_switch_sb_micbias(POWER_OFF);
        }

        if((g_record_wc & CODEC_MODE_WC_LINE_IN_V)){
            __dlv_disable_li_for_adc();
        }
    }

    g_is_recording = 0;
    return 0;
}

static int dlv_event_suspend(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);
    if(g_is_replaying){
        dlv_event_replay_stop();
    }
    if(g_is_recording){
        dlv_event_record_stop();
    }
    dlv_mode_ctrl(CODEC_MODE_OFF);
    dlv_shutdown();
    return 0;
}

static int dlv_event_resume(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);
    dlv_startup();
    dlv_mode_ctrl(g_current_mode);
    if(g_is_recording){
        dlv_event_record_start();
    }
    if(g_is_replaying){
        dlv_event_replay_start();
    }
    return 0;
}

//**************************************************************************
//  Provide interface to jz_snd.c
//**************************************************************************

/**
 * Set record or replay data width
 *
 * mode:    record or replay
 * width:	data width to set
 * return:	data width after set
 *
 * Provide data width of codec control interface for jz_snd.c driver
 */
static int dlv_dump_regs(const char *str)
{
	unsigned int i;
	unsigned char data;
    char* reg_names[]={"SR", "AICR_DAC", "AICR_ADC", "CR_LO", "CR_HP", "", "CR_DAC",
                       "CR_MIC", "CR_LI", "CR_ADC", "CR_MIX", "CR_VIC", "CCR",
                       "FCR_DAC", "FCR_ADC", "ICR", "IMR", "IFR", "GCR_HPL",
                       "GCR_HPR", "GCR_LIBYL", "GCR_LIBYR", "GCR_DACL", "GCR_DACR",
                       "GCR_MIC1", "GCR_MIC2", "GCR_ADCL", "GCR_ADCR", "", "GCR_MIXADC",
                       "GCR_MIXDAC", "AGC1", "AGC2", "AGC3", "AGC4", "AGC5"
    };
	printk("codec register dump, %s:\n", str);

    DLV_LOCK();
	for (i = 0; i < 36; i++) {
		data = dlv_read_reg(i);
		printk("address = 0x%02x, data = 0x%02x, name = %s\n", i, data, reg_names[i]);
	}
    DLV_UNLOCK();

    return 0;
}

/**
 * Set record or replay data width
 *
 * mode:    record or replay
 * width:	data width to set
 * return:	data width after set
 *
 * Provide data width of codec control interface for jz_snd.c driver
 */
static int dlv_set_width(int mode, int width)
{
	int supported_width[] = {16, 18, 20, 24};
	int i, wd = -1;

	DLV_LOCK();
	JZ4770_DLV_DEBUG_MSG("enter %s, mode = %d, width = %d\n", __func__, mode, width);
	if (width < 16)
		wd = 16;
	else if (width > 24)
		wd = 24;
	else {
		for (i = 0; i < ARRAY_SIZE(supported_width); i++) {
			if (supported_width[i] <= width)
				wd = supported_width[i];
			else
				break;
		}
	}

	if (mode & MODE_REPLAY) {
		switch(wd) {
		case 16:
			__dlv_dac_16bit_sample();
			break;
		case 18:
			__dlv_dac_18bit_sample();
			break;
		case 20:
			__dlv_dac_20bit_sample();
			break;
		case 24:
			__dlv_dac_24bit_sample();
			break;
		default:
			;
		}
	} 
    if (mode & MODE_RECORD){
		switch(wd) {
		case 16:
			__dlv_adc_16bit_sample();
			break;
		case 18:
			__dlv_adc_18bit_sample();
			break;
		case 20:
			__dlv_adc_20bit_sample();
			break;
		case 24:
			__dlv_adc_24bit_sample();
			break;
		default:
			;
		}
	}

    DLV_UNLOCK();
	return wd;
}

/**
 * Set record or replay rate
 *
 * mode:    record or replay
 * rate:	rate to set
 * return:	rate after set
 *
 * Provide rate of codec control interface for jz_snd.c driver
 */
static int dlv_set_rate(int mode, int rate)
{
	int speed = 0, val;
	int mrate[] = {
		96000, 48000, 44100, 32000, 24000,
		22050, 16000, 12000, 11025, 8000
	};

	JZ4770_DLV_DEBUG_MSG("enter %s, mode = %d, rate = %d\n", __func__, mode, rate);

	for (val = 0; val < ARRAY_SIZE(mrate); val++) {
		if (rate >= mrate[val]) {
			speed = val;
			break;
		}
	}
	if (rate < mrate[ARRAY_SIZE(mrate) - 1]) {
		speed = ARRAY_SIZE(mrate) - 1;
	}

	DLV_LOCK();
    if(mode & MODE_REPLAY){
        __dlv_set_dac_sample_rate(speed);        
    }
    if(mode & MODE_RECORD){
        __dlv_set_adc_sample_rate(speed);
    }
    DLV_UNLOCK();
	return mrate[speed];
}

/**
 * Set record or replay channels
 *
 * mode:    record or replay
 * channels:	channels to set
 * return:	channels after set
 *
 * Provide rate of codec control interface for jz_snd.c driver
 */
static int dlv_set_channels(int mode, int channels)
{
	JZ4770_DLV_DEBUG_MSG("enter %s, mode = %d, channels = %d\n", __func__, mode, channels);
	DLV_LOCK();

    if(mode & MODE_REPLAY){
        if(channels == 1){
            // MONO->1 for Mono
            __dlv_enable_dac_mono();
        }else{
            channels = 2;
            // MONO->0 for Stereo
            __dlv_disable_dac_mono();
        }
    }

    DLV_UNLOCK();
	return channels;
}

//FIXME:set volume depends on mode (replay and record)

/**
 * Set replay volume
 *
 * vol:	vol to set
 * return:	vol after set
 *
 * Provide volume of codec control interface for jz_snd.c driver
 */
static int dlv_set_replay_volume(int vol)
{
	unsigned int fixed_vol;

	JZ4770_DLV_DEBUG_MSG("enter %s, vol = %d\n", __func__, vol);

    if(vol < 0){
        vol = 0;
    }
    if(vol > 100){
        vol = 100;
    }

	DLV_LOCK();
	fixed_vol = 31 * (100 - vol) / 100;
	__dlv_set_hp_volume(fixed_vol);

    g_replay_volume = vol;
	if (vol == 0) {
		__dlv_set_dac_gain(0x1f);
        if(g_is_replaying){
            turn_off_dac(10);
        }
	} else {
		__dlv_set_dac_gain(0x06);
        if(g_is_replaying){
            turn_on_dac(10);
        }
	}

    DLV_UNLOCK();
    return vol;
}

/**
 * Set record volume
 *
 * vol:	vol to set
 * return:	vol after set
 *
 * Provide volume of codec control interface for jz_snd.c driver
 */
static int dlv_set_record_volume(int vol)
{
	unsigned int fixed_vol;

	JZ4770_DLV_DEBUG_MSG("enter %s, vol = %d\n", __func__, vol);

    if(vol < 0){
        vol = 0;
    }
    if(vol > 100){
        vol = 100;
    }

	DLV_LOCK();
	fixed_vol = 31 * vol / 100;
	__dlv_set_adc_gain(fixed_vol);
	__dlv_set_mic1_boost(0);

    DLV_UNLOCK();
    return 0;
}


static int dlv_set_mode(int mode)
{
    int ret = 0;
	JZ4770_DLV_DEBUG_MSG("enter %s, mode = 0x%x\n", __func__, mode);
	DLV_LOCK();
    ret = dlv_mode_ctrl(mode);
    DLV_UNLOCK();
    return ret;
}

static int dlv_set_gcr(struct codec_gcr_ctrl *ctrl)
{
    int ret = 0;
    char *name = ctrl->name;
    int gain1 = ctrl->gain1;
    int gain2 = ctrl->gain2;

    if(ctrl == NULL){
        return -1;
    }

	DLV_LOCK();
    if(strcmp(name, "GCR_HP") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 31){
            gain1 = 31;
        }
        __dlv_set_hp_volume(gain1);
    }else if(strcmp(name, "GCR_HPLR") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 31){
            gain1 = 31;
        }
        if(gain2 < 0){
            gain2 = 0;
        }
        if(gain2 > 31){
            gain2 = 31;
        }
        __dlv_set_hp_volume_lr(gain1, gain2);
    }else if(strcmp(name, "GCR_LIBY") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 31){
            gain1 = 31;
        }
        __dlv_set_line_in_bypass_volume(gain1);
    }else if(strcmp(name, "GCR_LIBYLR") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 31){
            gain1 = 31;
        }
        if(gain2 < 0){
            gain2 = 0;
        }
        if(gain2 > 31){
            gain2 = 31;
        }
        __dlv_set_line_in_bypass_volume_rl(gain1, gain2);
    }else if(strcmp(name, "GCR_DAC") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 31){
            gain1 = 31;
        }
        __dlv_set_dac_gain(gain1);
    }else if(strcmp(name, "GCR_DACLR") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 31){
            gain1 = 31;
        }
        if(gain2 < 0){
            gain2 = 0;
        }
        if(gain2 > 31){
            gain2 = 31;
        }
        __dlv_set_dac_gain_lr(gain1, gain2);
    }else if(strcmp(name, "GCR_MIC1") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 7){
            gain1 = 7;
        }
        __dlv_set_mic1_boost(gain1);
    }else if(strcmp(name, "GCR_MIC2") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 7){
            gain1 = 7;
        }
        __dlv_set_mic2_boost(gain2);
    }else if(strcmp(name, "GCR_ADC") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 63){
            gain1 = 63;
        }
        __dlv_set_adc_gain(gain1);
    }else if(strcmp(name, "GCR_ADCLR") == 0){
        if(gain1 < 0){
            gain1 = 0;
        }
        if(gain1 > 63){
            gain1 = 63;
        }
        if(gain2 < 0){
            gain2 = 0;
        }
        if(gain2 > 63){
            gain2 = 63;
        }
        __dlv_set_adc_gain_lr(gain1, gain2);
    }else{
        ret = -1;
    }
    DLV_UNLOCK();

    ctrl -> gain1 = gain1;
    ctrl -> gain2 = gain2;
    
    return ret;
}

/**
 * CODEC triggle  routine
 *
 * Provide event control interface for jz_snd.c driver
 */
static int dlv_triggle(int event)
{
    int ret;
	JZ4770_DLV_DEBUG_MSG("enter %s, event = %d\n", __func__, event);
	DLV_LOCK();

    ret = 0;
    switch(event){
    case CODEC_EVENT_INIT:
        ret = dlv_event_init();
		break;
    case CODEC_EVENT_DEINIT:
        ret = dlv_event_deinit();
		break;
    case CODEC_EVENT_SUSPEND:
        ret = dlv_event_suspend();
		break;
    case CODEC_EVENT_RESUME:
        ret = dlv_event_resume();
		break;
    case CODEC_EVENT_REPLAY_START:
        ret = dlv_event_replay_start();
		break;
    case CODEC_EVENT_REPLAY_STOP:
        ret = dlv_event_replay_stop();
		break;
    case CODEC_EVENT_RECORD_START:
        ret = dlv_event_record_start();
		break;
    case CODEC_EVENT_RECORD_STOP:
        ret = dlv_event_record_stop();
		break;
    default:
        break;
    }

    DLV_UNLOCK();
    return ret;
}

//**************************************************************************
//  Mudule
//**************************************************************************

static struct codec_ops codec_ops = {
    .dump_regs    = dlv_dump_regs,
    .set_width    = dlv_set_width,
    .set_rate     = dlv_set_rate,
    .set_channels = dlv_set_channels,
    .set_replay_volume  = dlv_set_replay_volume,
    .set_record_volume  = dlv_set_record_volume,
    .set_gcr = dlv_set_gcr,
    .triggle  = dlv_triggle,
    .set_mode = dlv_set_mode,
};


/**
 * Module init
 */
static int __init dlv_init(void)
{
	int retval;

	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);

	spin_lock_init(&dlv_irq_lock);
	DLV_LOCKINIT();
    DLV_UNLOCK();

	register_codec(&codec_ops);

    //register irq
	INIT_WORK(&dlv_irq_work, dlv_irq_work_handler);
	retval = request_irq(IRQ_AIC, dlv_codec_irq, IRQF_DISABLED, "dlv_codec_irq", NULL);
	if (retval) {
		printk("JZ DLV: Could not get AIC CODEC irq %d\n", IRQ_AIC);
		return retval;
	}

	return 0;
}

/**
 * Module exit
 */
static void __exit dlv_exit(void)
{
	JZ4770_DLV_DEBUG_MSG("enter %s\n", __func__);

	unregister_codec(&codec_ops);
	free_irq(IRQ_AIC, NULL);

	DLV_LOCKDEINIT();
}

module_init(dlv_init);
module_exit(dlv_exit);

MODULE_AUTHOR("liulu <lliu@ingenic.cn>");
MODULE_DESCRIPTION("jz4770 internel codec driver");
MODULE_LICENSE("GPL");
