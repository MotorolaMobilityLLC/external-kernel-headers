/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#ifndef _SPRD_DCAM_H_
#define _SPRD_DCAM_H_

/*  Four-character-code (FOURCC) */
#define img_fourcc(a, b, c, d)\
	((unsigned int)(a) | ((unsigned int)(b) << 8) | ((unsigned int)(c) << 16) | ((unsigned int)(d) << 24))

/* RGB formats */
#define IMG_PIX_FMT_RGB565  img_fourcc('R', 'G', 'B', 'P') /* 16  RGB-5-6-5     */
#define IMG_PIX_FMT_RGB565X img_fourcc('R', 'G', 'B', 'R') /* 16  RGB-5-6-5 BE  */

/* Grey formats */
#define IMG_PIX_FMT_GREY    img_fourcc('G', 'R', 'E', 'Y') /*  8  Greyscale     */

/* Luminance+Chrominance formats */
#define IMG_PIX_FMT_YVU420  img_fourcc('Y', 'V', '1', '2') /* 12  YVU 4:2:0     */
#define IMG_PIX_FMT_YUYV    img_fourcc('Y', 'U', 'Y', 'V') /* 16  YUV 4:2:2     */
#define IMG_PIX_FMT_YVYU    img_fourcc('Y', 'V', 'Y', 'U') /* 16 YVU 4:2:2 */
#define IMG_PIX_FMT_UYVY    img_fourcc('U', 'Y', 'V', 'Y') /* 16  YUV 4:2:2     */
#define IMG_PIX_FMT_VYUY    img_fourcc('V', 'Y', 'U', 'Y') /* 16  YUV 4:2:2     */
#define IMG_PIX_FMT_YUV422P img_fourcc('4', '2', '2', 'P') /* 16  YVU422 planar */
#define IMG_PIX_FMT_YUV420  img_fourcc('Y', 'U', '1', '2') /* 12  YUV 4:2:0     */

/* two planes -- one Y, one Cr + Cb interleaved  */
#define IMG_PIX_FMT_NV12    img_fourcc('N', 'V', '1', '2') /* 12  Y/CbCr 4:2:0  */
#define IMG_PIX_FMT_NV21    img_fourcc('N', 'V', '2', '1') /* 12  Y/CrCb 4:2:0  */

/* compressed formats */
#define IMG_PIX_FMT_JPEG     img_fourcc('J', 'P', 'E', 'G') /* JFIF JPEG     */

#define SPRD_IMG_PATH_MAX    6

#define SPRD_FLASH_MAX_CELL  40


enum {
	DCAM_TX_DONE       = 0x00,
	DCAM_NO_MEM        = 0x01,
	DCAM_TX_ERR        = 0x02,
	DCAM_CSI2_ERR      = 0x03,
	DCAM_SYS_BUSY      = 0x04,
	DCAM_CANCELED_BUF  = 0x05,
	DCAM_TIMEOUT       = 0x10,
	DCAM_TX_STOP       = 0xFF
};

enum {
	IMG_ENDIAN_BIG = 0,
	IMG_ENDIAN_LITTLE,
	IMG_ENDIAN_HALFBIG,
	IMG_ENDIAN_HALFLITTLE,
	IMG_ENDIAN_MAX
};

enum if_status {
	IF_OPEN = 0,
	IF_CLOSE
};

enum {
	SPRD_IMG_GET_SCALE_CAP = 0,
	SPRD_IMG_GET_FRM_BUFFER,
	SPRD_IMG_STOP_DCAM,
	SPRD_IMG_FREE_FRAME,
	SPRD_IMG_GET_PATH_CAP
};

enum sprd_flash_type {
	FLASH_TYPE_PREFLASH,
	FLASH_TYPE_MAIN,
	FLASH_TYPE_MAX
};

enum sprd_flash_io_id {
	FLASH_IOID_GET_CHARGE,
	FLASH_IOID_GET_TIME,
	FLASH_IOID_GET_MAX_CAPACITY,
	FLASH_IOID_SET_CHARGE,
	FLASH_IOID_SET_TIME,
	FLASH_IOID_MAX
};

/* rotation */
enum {
	ROT_YUV422 = 0,
	ROT_YUV420,
	ROT_YUV400,
	ROT_RGB888,
	ROT_RGB666,
	ROT_RGB565,
	ROT_RGB555,
	ROT_FMT_MAX
};

enum {
	ROT_90 = 0,
	ROT_270,
	ROT_180,
	ROT_MIRROR,
	ROT_ANGLE_MAX
};

enum {
	ROT_ENDIAN_BIG = 0,
	ROT_ENDIAN_LITTLE,
	ROT_ENDIAN_HALFBIG,
	ROT_ENDIAN_HALFLITTLE,
	ROT_ENDIAN_MAX
};

/* scaling */
enum {
	SCALE_YUV422 = 0,
	SCALE_YUV420,
	SCALE_YUV400,
	SCALE_YUV420_3FRAME,
	SCALE_RGB565,
	SCALE_RGB888,
	SCALE_FTM_MAX
};

enum {
	SCALE_ENDIAN_BIG = 0,
	SCALE_ENDIAN_LITTLE,
	SCALE_ENDIAN_HALFBIG,
	SCALE_ENDIAN_HALFLITTLE,
	SCALE_ENDIAN_MAX
};

enum {
	SCALE_MODE_NORMAL = 0,
	SCALE_MODE_SLICE,
	SCALE_MODE_SLICE_READDR,
	SCALE_MODE_MAX
};

enum {
	MM_PW_DOMAIN_DCAM = 0,
	MM_PW_DOMAIN_ISP,
	MM_PW_DOMAIN_JPG,
	MM_PW_DOMAIN_COUNT_MAX
};

struct sprd_dcam_size {
	unsigned int w;
	unsigned int h;
};

struct sprd_dcam_rect {
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
};

struct sprd_dcam_frm_addr {
	unsigned int y;
	unsigned int u;
	unsigned int v;
};

struct sprd_dcam_parm {
	unsigned int channel_id;
	unsigned int frame_base_id;
	unsigned int                  pixel_fmt;
	unsigned int                  need_isp_tool;
	unsigned int                  deci;
	unsigned int                  shrink;
	unsigned int                  index;
	unsigned int                  need_isp;
	unsigned int                  is_reserved_buf;

	struct sprd_dcam_rect crop_rect;
	struct sprd_dcam_size dst_size;
	struct sprd_dcam_frm_addr addr;
	struct sprd_dcam_frm_addr addr_vir;

	unsigned int reserved[4];
};

struct sprd_img_ccir_if {
	unsigned int v_sync_pol;
	unsigned int h_sync_pol;
	unsigned int pclk_pol;
	unsigned int res1;
	unsigned int padding;
};

struct sprd_img_mipi_if {
	unsigned int use_href;
	unsigned int bits_per_pxl;
	unsigned int is_loose;
	unsigned int lane_num;
	unsigned int pclk;
};

struct sprd_img_sensor_if {
	unsigned int if_type;
	unsigned int img_fmt;
	unsigned int img_ptn;
	unsigned int frm_deci;
	unsigned int res[4];
	union {
		struct sprd_img_ccir_if ccir;
		struct sprd_img_mipi_if mipi;
	} if_spec;
};

struct sprd_img_frm_info {
	unsigned int channel_id;
	unsigned int height;
	unsigned int length;
	unsigned int sec;
	unsigned int usec;
	unsigned int frm_base_id;
	unsigned int index;
	unsigned int real_index;
	unsigned int img_fmt;

	struct sprd_dcam_frm_addr addr;
	struct sprd_dcam_frm_addr addr_vir;
	unsigned int reserved[4];
};

struct sprd_img_path_info {
	unsigned int               line_buf;
	unsigned int               support_yuv;
	unsigned int               support_raw;
	unsigned int               support_jpeg;
	unsigned int               support_scaling;
	unsigned int               support_trim;
	unsigned int               is_scaleing_path;;
};

struct sprd_img_path_capability {
	unsigned int               count;
	struct sprd_img_path_info  path_info[SPRD_IMG_PATH_MAX];
};

struct sprd_img_write_op {
	unsigned int cmd;
	unsigned int channel_id;
	unsigned int index;
};

struct sprd_img_read_op {
	unsigned int cmd;
	unsigned int evt;
	union {
		struct sprd_img_frm_info frame;
		struct sprd_img_path_capability capability;
		unsigned int reserved[20];
	} parm;
};

struct sprd_img_get_fmt {
	unsigned int index;
	unsigned int fmt;
};

struct sprd_dcam_time {
	unsigned int sec;
	unsigned int usec;
};

struct sprd_img_format {
	unsigned int channel_id;
	unsigned int width;
	unsigned int height;
	unsigned int fourcc;
	unsigned int need_isp;
	unsigned int need_binning;
	unsigned int bytesperline;
	unsigned int is_lightly;
	unsigned int reserved[4];
};

struct sprd_flash_element {
	uint16_t index;
	uint16_t val;
};

struct sprd_flash_cell {
	uint8_t type;
	uint8_t count;
	uint8_t def_val;
	struct sprd_flash_element element[SPRD_FLASH_MAX_CELL];
};

struct sprd_flash_capacity {
	uint16_t max_charge;
	uint16_t max_time;
};

struct sprd_flash_cfg_param {
	unsigned int io_id;
	void *data;
};

/* rotation */
struct sprd_dcam_rot_cfg_parm {
	struct sprd_dcam_size size;
	unsigned int format;
	unsigned int angle;
	struct sprd_dcam_frm_addr src_addr;
	struct sprd_dcam_frm_addr dst_addr;
	unsigned int src_endian;
	unsigned int dst_endian;
};

/* scaling */
struct sprd_dcam_scale_endian_sel {
	unsigned char y_endian;
	unsigned char uv_endian;
	unsigned char reserved[2];
};

struct sprd_dcam_scale_slice_parm {
	unsigned int slice_height;
	struct sprd_dcam_rect input_rect;
	struct sprd_dcam_frm_addr input_addr;
	struct sprd_dcam_frm_addr output_addr;
};

struct sprd_dcam_scale_cfg_parm {
	struct sprd_dcam_size input_size;
	struct sprd_dcam_rect input_rect;
	unsigned int input_format;
	struct sprd_dcam_frm_addr input_addr;
	struct sprd_dcam_scale_endian_sel input_endian;

	struct sprd_dcam_size output_size;
	unsigned int output_format;
	struct sprd_dcam_frm_addr output_addr;
	struct sprd_dcam_scale_endian_sel output_endian;

	unsigned int scale_mode;
	unsigned int slice_height;
};


#define SPRD_DCAM_IO_MAGIC            'Z'
#define SPRD_DCAM_IO_SET_MODE          _IOW(SPRD_DCAM_IO_MAGIC, 0, unsigned int)
#define SPRD_DCAM_IO_SET_SKIP_NUM      _IOW(SPRD_DCAM_IO_MAGIC, 1, unsigned int)
#define SPRD_DCAM_IO_SET_SENSOR_SIZE   _IOW(SPRD_DCAM_IO_MAGIC, 2, struct sprd_dcam_size)
#define SPRD_DCAM_IO_SET_SENSOR_TRIM   _IOW(SPRD_DCAM_IO_MAGIC, 3, struct sprd_dcam_rect)
#define SPRD_DCAM_IO_SET_FRM_ID_BASE   _IOW(SPRD_DCAM_IO_MAGIC, 4, struct sprd_dcam_parm)
#define SPRD_DCAM_IO_SET_CROP          _IOW(SPRD_DCAM_IO_MAGIC, 5, struct sprd_dcam_parm)
#define SPRD_DCAM_IO_SET_FLASH         _IOW(SPRD_DCAM_IO_MAGIC, 6, unsigned int)
#define SPRD_DCAM_IO_SET_ZOOM_MODE     _IOW(SPRD_DCAM_IO_MAGIC, 7, unsigned int)
#define SPRD_DCAM_IO_SET_SENSOR_IF     _IOW(SPRD_DCAM_IO_MAGIC, 8, struct sprd_img_sensor_if)
#define SPRD_DCAM_IO_SET_FRAME_ADDR    _IOW(SPRD_DCAM_IO_MAGIC, 9, struct sprd_dcam_parm)
#define SPRD_DCAM_IO_PATH_FRM_DECI     _IOW(SPRD_DCAM_IO_MAGIC, 10, struct sprd_dcam_parm)
#define SPRD_DCAM_IO_PATH_PAUSE        _IOW(SPRD_DCAM_IO_MAGIC, 11, struct sprd_dcam_parm)
#define SPRD_DCAM_IO_PATH_RESUME       _IOW(SPRD_DCAM_IO_MAGIC, 12, unsigned int)
#define SPRD_DCAM_IO_STREAM_ON         _IOW(SPRD_DCAM_IO_MAGIC, 13, unsigned int)
#define SPRD_DCAM_IO_STREAM_OFF        _IOW(SPRD_DCAM_IO_MAGIC, 14, unsigned int)
#define SPRD_DCAM_IO_GET_FMT           _IOWR(SPRD_DCAM_IO_MAGIC, 15, struct sprd_img_get_fmt)
#define SPRD_DCAM_IO_GET_TIME          _IOR(SPRD_DCAM_IO_MAGIC, 16, struct sprd_dcam_time)
#define SPRD_DCAM_IO_CHECK_FMT         _IOWR(SPRD_DCAM_IO_MAGIC, 17, struct sprd_img_format)
#define SPRD_DCAM_IO_SET_SHRINK        _IOW(SPRD_DCAM_IO_MAGIC, 18, unsigned int)
#define SPRD_DCAM_IO_CFG_FLASH         _IOW(SPRD_DCAM_IO_MAGIC, 19, struct sprd_flash_cfg_param)
#define SPRD_DCAM_IO_GET_FREE_CH       _IOWR(SPRD_DCAM_IO_MAGIC, 20, struct sprd_dcam_parm)

/* rotation */
#define SPRD_DCAM_IO_OPEN_ROT          _IOW(SPRD_DCAM_IO_MAGIC, 21, unsigned int)
#define SPRD_DCAM_IO_CLOSE_ROT         _IOW(SPRD_DCAM_IO_MAGIC, 22, unsigned int)
#define SPRD_DCAM_IO_START_ROT         _IOW(SPRD_DCAM_IO_MAGIC, 23, struct sprd_dcam_rot_cfg_parm)

/* scaling */
#define SPRD_DCAM_IO_OPEN_SCALE        _IOW(SPRD_DCAM_IO_MAGIC, 24, unsigned int)
#define SPRD_DCAM_IO_CLOSE_SCALE       _IOW(SPRD_DCAM_IO_MAGIC, 25, unsigned int)
#define SPRD_DCAM_IO_START_SCALE       _IOW(SPRD_DCAM_IO_MAGIC, 26, struct sprd_dcam_scale_cfg_parm)
#define SPRD_DCAM_IO_CONTINUE_SCALE    _IOW(SPRD_DCAM_IO_MAGIC, 27, struct sprd_dcam_scale_slice_parm)
#define SPRD_DCAM_IO_STOP_SCALE        _IOW(SPRD_DCAM_IO_MAGIC, 28, unsigned int)

int mm_pw_on(unsigned int client);
int mm_pw_off(unsigned int client);

#endif

