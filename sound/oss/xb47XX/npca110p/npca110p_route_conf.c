/*
 * Linux/sound/oss/npca110p_route_conf.h
 *
 * DLV CODEC driver for Ingenic Jz4760 MIPS processor
 *
 * 2010-11-xx   jbbi <jbbi@ingenic.cn>
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include "npca110p_route_conf.h"

/***************************************************************************************\
 *                                                                                     *
 *typical config for each route                                                        *
 *                                                                                     *
\***************************************************************************************/
route_conf_base const replay_hp_stereo_conf = {
	.route_lineout_mode = LINEOUT_STEREO,
};

route_conf_base const bypass_linein_to_out_conf = {
	.route_linein_mode = LINEIN_WITH_BYPASS,
};

route_conf_base const route_all_clear_conf = {
};

struct __dlv_route_info dlv_route_info[] = {

	/************************ route clear ************************/
	{
		.route_name = ROUTE_ALL_CLEAR,
		.route_conf = &route_all_clear_conf,
	},
	{
		.route_name = REPLAY_HP_STEREO,
		.route_conf = &replay_hp_stereo_conf,
	},
	{
		.route_name = BYPASS_LINEIN_TO_OUT,
		.route_conf = &bypass_linein_to_out_conf,
	},
};
