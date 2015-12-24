/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _UAPI_VIDEO_GSP2L_CFG_H
#define _UAPI_VIDEO_GSP2L_CFG_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include "gsp_cfg.h"

struct gsp2l_img_layer_params {
	uint32_t			pitch;
	struct gsp_rect                 clip_rect;
	struct gsp_rect                 des_rect;
	struct gsp_rgb                  grey;
	struct gsp_rgb                  colorkey;
	struct gsp_endian		endian;
	enum gsp_src_layer_format       img_format;
	enum gsp_rot_angle		rot_angle;
	uint8_t                         row_tap_mode;
	uint8_t                         col_tap_mode;
	uint8_t                         alpha;
	uint8_t				colorkey_en;
	uint8_t				pallet_en;
	uint8_t				scaling_en;
	uint8_t				pmargb_en;
	uint8_t				pmargb_mod;
};

struct gsp2l_img_layer_user {
	struct gsp_layer_user		common;
	struct gsp2l_img_layer_params	params;
};

struct gsp2l_osd_layer_params {
	uint32_t                        pitch;
	struct gsp_rect                 clip_rect;
	struct gsp_pos                  des_pos;
	struct gsp_rgb                  grey;
	struct gsp_rgb                  colorkey;
	struct gsp_endian		endian;
	enum gsp_src_layer_format	osd_format;
	enum gsp_rot_angle		rot_angle;
	uint8_t                         row_tap_mode;
	uint8_t                         col_tap_mode;
	uint8_t				alpha;
	uint8_t				colorkey_en;
	uint8_t				pallet_en;
	uint8_t				pmargb_en;
	uint8_t				pmargb_mod;
};

struct gsp2l_osd_layer_user {
	struct gsp_layer_user		common;
	struct gsp2l_osd_layer_params	params;
};

struct gsp2l_des_layer_params {
	uint32_t                        pitch;
	struct gsp_endian		endian;
	enum gsp_des_layer_format       img_format;
	uint8_t				compress_r8_en;
};

struct gsp2l_des_layer_user {
	struct gsp_layer_user		common;
	struct gsp2l_des_layer_params	params;
};

struct gsp2l_cfg_user {
	struct gsp2l_img_layer_user l0;
	struct gsp2l_osd_layer_user l1;
	struct gsp2l_des_layer_user ld;
};

struct gsp2l_capability {
	struct gsp_capability common;
	/* 1: means 1/16, 64 means 4*/
	uint16_t scale_range_up;
	/* 1: means 1/16, 64 means 4*/
	uint16_t scale_range_down;
	int yuv_xywh_even;
	int max_video_size;
};

#endif
