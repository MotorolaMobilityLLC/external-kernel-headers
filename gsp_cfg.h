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

#ifndef _UAPI_VIDEO_GSP_CFG_H
#define _UAPI_VIDEO_GSP_CFG_H

#include <linux/ioctl.h>
#include <linux/types.h>


/* define ioctl code for gsp */
#define GSP_IO_MAGIC  ('G')

#define GSP_IO_SHIFT  (5)

#define GSP_GET_CAPABILITY_SHIFT  (6)
#define GSP_TRIGGER_SHIFT  (5)
#define GSP_ASYNC_SHIFT  (4)
#define GSP_SPLIT_SHIFT  (3)
#define GSP_CNT_SHIFT  (0)

/*
 * _IO_NR() has 8 bits
 * bit7: MAX_NR indicate max validate code
 * bit6: indicate GSP_GET_CAPABILTY code
 * bit5: indicate GSP_TRIGGER code
 * bit4: indicate whether kcfgs are async
 * bit3: indicate whether kcfgs are split which
 *	 respond to split-case
 * bit2-bit0: indicate how many kcfgs there are
 */
#define GSP_GET_CAPABILITY  (0x1 << GSP_GET_CAPABILITY_SHIFT)
#define GSP_TRIGGER  (0x1 << GSP_TRIGGER_SHIFT)
#define GSP_IO_MASK  (0x7 << GSP_IO_SHIFT)
#define GSP_ASYNC_MASK (0x1 << GSP_ASYNC_SHIFT)
#define GSP_SPLIT_MASK (0x1 << GSP_SPLIT_SHIFT)
#define GSP_CNT_MASK (0x7 << GSP_CNT_SHIFT)

#define GSP_IO_GET_CAPABILITY(size)  \
_IOWR(GSP_IO_MAGIC, GSP_GET_CAPABILITY, size)

#define GSP_IO_TRIGGER(async, cnt, split, size)  \
{\
_IOWR(GSP_IO_MAGIC,\
GSP_TRIGGER | (async) << GSP_ASYNC_SHIFT |\
(split) << GSP_SPLIT_SHIFT | (cnt) << GSP_CNT_SHIFT,\
size)\
}

#define GSP_CAPABILITY_MAGIC  0xDEEFBEEF

enum gsp_layer_type {
	GSP_IMG_LAYER,
	GSP_OSD_LAYER,
	GSP_DES_LAYER,
	GSP_INVAL_LAYER
};

/*the address type of gsp can process*/
enum gsp_addr_type {
	GSP_ADDR_TYPE_INVALUE,
	GSP_ADDR_TYPE_PHYSICAL,
	GSP_ADDR_TYPE_IOVIRTUAL,
	GSP_ADDR_TYPE_MAX,
};

enum gsp_irq_mod {
	GSP_IRQ_MODE_PULSE = 0x00,
	GSP_IRQ_MODE_LEVEL,
	GSP_IRQ_MODE_LEVEL_INVALID,
};

enum gsp_irq_type {
	GSP_IRQ_TYPE_DISABLE = 0x00,
	GSP_IRQ_TYPE_ENABLE,
	GSP_IRQ_TYPE_INVALID,
};

enum GSP_RNT {
	GSP_RTN_SUCCESS = 0x00,
	GSP_RTN_POINTER_INVALID,
	GSP_RTN_PARAM_INVALID,
};

enum gsp_rot_angle {
	GSP_ROT_ANGLE_0 = 0x00,
	GSP_ROT_ANGLE_90,
	GSP_ROT_ANGLE_180,
	GSP_ROT_ANGLE_270,
	GSP_ROT_ANGLE_0_M,
	GSP_ROT_ANGLE_90_M,
	GSP_ROT_ANGLE_180_M,
	GSP_ROT_ANGLE_270_M,
	GSP_ROT_ANGLE_MAX_NUM,
};

/*Original: B3B2B1B0*/
enum gsp_word_endian {
	GSP_WORD_ENDN_0 = 0x00,     /*B3B2B1B0*/
	GSP_WORD_ENDN_1,            /*B0B1B2B3*/
	GSP_WORD_ENDN_2,            /*B2B3B0B1*/
	GSP_WORD_ENDN_3,            /*B1B0B3B2*/
	GSP_WORD_ENDN_MAX_NUM,
};

enum gsp_rgb_swap_mod {
	GSP_RGB_SWP_RGB = 0x00,
	GSP_RGB_SWP_RBG,
	GSP_RGB_SWP_GRB,
	GSP_RGB_SWP_GBR,
	GSP_RGB_SWP_BGR,
	GSP_RGB_SWP_BRG,
	GSP_RGB_SWP_MAX,
};

enum gsp_a_swap_mod {
	GSP_A_SWAP_ARGB,
	GSP_A_SWAP_RGBA,
	GSP_A_SWAP_MAX,
};

enum gsp_data_format {
	GSP_FMT_ARGB888 = 0x00,
	GSP_FMT_RGB888,
	GSP_FMT_CMPESS_RGB888,
	GSP_FMT_ARGB565,
	GSP_FMT_RGB565,
	GSP_FMT_YUV420_2P,
	GSP_FMT_YUV420_3P,
	GSP_FMT_YUV400,
	GSP_FMT_YUV422,
	GSP_FMT_8BPP,
	GSP_FMT_PMARGB,
	GSP_FMT_MAX_NUM,
};


enum gsp_src_layer_format {
	GSP_SRC_FMT_ARGB888 = 0x00,
	GSP_SRC_FMT_RGB888,
	GSP_SRC_FMT_ARGB565,
	GSP_SRC_FMT_RGB565,
	GSP_SRC_FMT_YUV420_2P,
	GSP_SRC_FMT_YUV420_3P,
	GSP_SRC_FMT_YUV400_1P,
	GSP_SRC_FMT_YUV422_2P,
	GSP_SRC_FMT_8BPP,
	GSP_SRC_FMT_MAX_NUM,
};

enum gsp_des_layer_format {
	GSP_DST_FMT_ARGB888 = 0x00,
	GSP_DST_FMT_RGB888,
	GSP_DST_FMT_ARGB565,
	GSP_DST_FMT_RGB565,
	GSP_DST_FMT_YUV420_2P,
	GSP_DST_FMT_YUV420_3P,
	GSP_DST_FMT_YUV422_2P,
	GSP_DST_FMT_MAX_NUM,
};

struct gsp_endian {
	enum gsp_word_endian             y_word_endn;
	enum gsp_word_endian             uv_word_endn;
	enum gsp_word_endian             va_word_endn;
	enum gsp_rgb_swap_mod          rgb_swap_mode;
	enum gsp_a_swap_mod            a_swap_mode;
};

struct gsp_rgb {
	uint8_t b_val;
	uint8_t g_val;
	uint8_t r_val;
	uint8_t a_val;
};

struct gsp_pos {
	uint16_t pt_x;
	uint16_t pt_y;
};

struct gsp_rect {
	uint16_t st_x;
	uint16_t st_y;
	uint16_t rect_w;
	uint16_t rect_h;
};

struct gsp_addr_data {
	__u32 addr_y;
	__u32 addr_uv;
	__u32 addr_va;
};

struct gsp_offset {
	__u32 uv_offset;
	__u32 v_offset;
};

/*
 * to distinguish struct from uapi gsp cfg header file
 * and no uapi gsp cfg header file. structure at uapi
 * header file has suffix "_user"
 */

struct gsp_layer_user {
	int type;
	int enable;
	int share_fd;
	int wait_fd;
	int sig_fd;
	struct gsp_addr_data src_addr;
	struct gsp_offset offset;
};


#define CAPABILITY_MAGIC_NUMBER 0xDEEFBEEF
struct gsp_capability {
	/*used to indicate struct is initialized*/
	uint32_t magic;
	char version[32];

	size_t capa_size;
	uint32_t io_cnt;
	uint32_t core_cnt;

	uint32_t max_layer;
	uint32_t max_img_layer;

	struct gsp_rect crop_max;
	struct gsp_rect crop_min;
	struct gsp_rect out_max;
	struct gsp_rect out_min;

	/* GSP_ADDR_TYPE_PHYSICAL:phy addr
	 * GSP_ADDR_TYPE_IOVIRTUAL:iova addr*/
	enum gsp_addr_type buf_type;
};

#endif
