/*
 * linux/arch/mips/jz4760/cpm_fake.c
 *
 * jz4760 on-chip modules.
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: <whxu@ingenic.cn>
 */

#include <asm/jzsoc.h>


#ifndef JZ_EXTAL
#define JZ_EXTAL        (12 * 1000000)  /* 12MHz */
#endif


/*
 * Get the external clock
 */
static unsigned int get_external_clock(void)
{
        return JZ_EXTAL;
}

/*
 * Get the PLL clock
 */
unsigned int cpm_get_pllout(void)
{
        return get_external_clock() * CFG_DIV;
}

/*
 * Get the PLL2 clock
 */
unsigned int cpm_get_pllout1(void)
{
        return get_external_clock();
}

/*
 * Start the module clock
 */
void cpm_start_clock(clock_gate_module module_name)
{

}

/*
 * Stop the module clock
 */
void cpm_stop_clock(clock_gate_module module_name)
{

}

/*
 * Get the clock, assigned by the clock_name, and the return value unit is Hz
 */
unsigned int cpm_get_clock(cgu_clock clock_name)
{
        unsigned int clock_hz;

        switch (clock_name) {
        case CGU_CCLK:
                clock_hz = cpm_get_pllout();

                break;

        case CGU_HCLK:
                clock_hz = get_external_clock();

                break;

        case CGU_PCLK:
                clock_hz = get_external_clock() / CFG_DIV;

                break;

        case CGU_MCLK:
                clock_hz = get_external_clock() / CFG_DIV;

                break;

        case CGU_H2CLK:
                clock_hz = get_external_clock();

                break;

        case CGU_SCLK:
        case CGU_MSCCLK:
        case CGU_SSICLK:
        case CGU_CIMCLK:
        case CGU_LPCLK:
	case CGU_TVECLK:
        case CGU_I2SCLK:
        case CGU_PCMCLK:
        case CGU_OTGCLK:
        case CGU_UHCCLK:
        case CGU_GPSCLK:
        case CGU_GPUCLK:
        case CGU_UARTCLK:
        case CGU_SADCCLK:
        case CGU_TCUCLK:
                clock_hz = get_external_clock() / CFG_DIV;

                break;

        default:
                printk("WARNING: can NOT get clock %d!\n", clock_name);
                clock_hz = get_external_clock();
                break;
        }

        return clock_hz;
}

/*
 * Set the clock, assigned by the clock_name, and the return value unit is Hz,
 * which means the actual clock
 */
unsigned int cpm_set_clock(cgu_clock clock_name, unsigned int clock_hz)
{
        return 0;
}
