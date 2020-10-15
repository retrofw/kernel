/*
 * Ingenic JZ475X Display Controllers Driver.
 * 
 * Output Main Path.
 * 
 * Copyright (C) 2005-2010 Ingenic Semiconductor Inc.
 * Author: River Wang <zwang@ingenic.cn>
 *		
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include "control.c"

#ifdef CONFIG_FB_JZ475X_LCD_OUTPUT
#include "lcd/lcd-output.c"
#endif

#ifdef CONFIG_FB_JZ475X_TVE_PAL_OUTPUT
#include "tve/tve-pal-output.c"
#endif

#ifdef CONFIG_FB_JZ475X_TVE_NTSC_OUTPUT
#include "tve/tve-ntsc-output.c"
#endif


