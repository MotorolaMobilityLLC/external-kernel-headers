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

#ifndef _ISP_DRV_KERNEL_H_
#define _ISP_DRV_KERNEL_H_
#include <linux/time.h>

#define IMG_BUF_NUM_MAX                         4
#define MAXSEQLEN                               20

enum isp_img_output_id {
	ISP_IMG_PREVIEW = 0,
	ISP_IMG_VIDEO,
	ISP_IMG_STILL_CAPTURE,
	ISP_IMG_STATISTICS,
	ISP_OUTPUT_IMG_TOTAL,
};

enum isp_rtn{
	ISP_IMG_TX_DONE = 0x20,
	ISP_IMG_NO_MEM,
	ISP_IMG_TX_ERR,
	ISP_IMG_SYS_BUSY,
	ISP_IMG_TIMEOUT,
	ISP_IMG_TX_STOP,
};

enum isp_output_img_format {
	ISP_OUT_IMG_RAW10 = 0,
	ISP_OUT_IMG_NV12,
	ISP_OUT_IMG_YUY2,
	ISP_OUT_IMG_FORMAT_TOTAL,
};

enum {
	ISP_INT_NONE                     = 0,
	// System INT Boot finish,Cmd finish, IP error
	ISP_SYS_BOOT_FINISH_INT          = 0x00000001<<0,//0x00000001
	ISP_SYS_CMD_FINISH_INT           = 0x00000001<<1,//0x00000002
	ISP_SYS_IP_ERROR_INT             = 0x00000001<<2,//0x00000004
	// Sensor1 output img1, Img2, Img3 done
	ISP_SENSOR1_OUT_IMG1_DONE_INT    = 0x00000001<<3,//0x00000008
	ISP_SENSOR1_OUT_IMG2_DONE_INT    = 0x00000001<<4,//0x00000010
	ISP_SENSOR1_OUT_IMG3_DONE_INT    = 0x00000001<<5,//0x00000020
	// Sensor1 3A done, AF lock, AE lock
	ISP_SENSOR1_3A_DONE_INT          = 0x00000001<<6,//0x00000040
	ISP_SENSOR1_3A_SOF_INT           = 0x00000001<<7,//0x00000080
	ISP_SENSOR1_3A_LINEMEET_INT      = 0x00000001<<8,//0x00000100
	// Sensor2 output img1, img2, img3 done
	ISP_SENSOR2_OUT_IMG1_DONE_INT    = 0x00000001<<9,//0x00000200
	ISP_SENSOR2_OUT_IMG2_DONE_INT    = 0x00000001<<10,//0x00000400
	ISP_SENSOR2_OUT_IMG3_DONE_INT    = 0x00000001<<11,//0x00000800
	// Sensor2 3A done, AF lock, AE lock
	ISP_SENSOR2_3A_DONE_INT          = 0x00000001<<12,//0x00001000
	ISP_SENSOR2_3A_SOF_INT           = 0x00000001<<13,//0x00002000
	ISP_SENSOR2_3A_LINEMEET_INT      = 0x00000001<<14,//0x00004000
	// Sensor2 output img done
	ISP_SENSOR3_OUT_IMG_DONE_INT     = 0x00000001<<15,//0x00008000
	// Sensor3 3A done, AF lock, AE lock
	ISP_SENSOR3_3A_DONE_INT          = 0x00000001<<16,//0x00010000
	ISP_SENSOR3_3A_SOF_INT           = 0x00000001<<17,//0x00020000
	ISP_SENSOR3_3A_LINEMEET_INT      = 0x00000001<<18,//0x00040000
};

enum {
	ISP_SHARPNESS_LV0 = 0, // -2
	ISP_SHARPNESS_LV1, // -1.5
	ISP_SHARPNESS_LV2, // -1
	ISP_SHARPNESS_LV3, // -0.5
	ISP_SHARPNESS_LV4, // 0
	ISP_SHARPNESS_LV5, // 0.5
	ISP_SHARPNESS_LV6, // 1
	ISP_SHARPNESS_LV7, // 1.5
	ISP_SHARPNESS_LV8, // 2
	ISP_SHARPNESS_TOTAL
};

enum {
	ISP_SATURATION_LV0, // -2
	ISP_SATURATION_LV1, // -1.5
	ISP_SATURATION_LV2, // -1
	ISP_SATURATION_LV3, // -0.5
	ISP_SATURATION_LV4, // 0
	ISP_SATURATION_LV5, // 0.5
	ISP_SATURATION_LV6, // 1
	ISP_SATURATION_LV7, // 1.5
	ISP_SATURATION_LV8, // 2
	// ISP_SATURATION_NORMAL,
	// ISP_SATURATION_LOW,
	// ISP_SATURATION_HIGH,
	ISP_SATURATION_TOTAL
};

enum {
	ISP_SPECIAL_EFFECT_OFF,
	ISP_SPECIAL_EFFECT_GRAYSCALE,
	ISP_SPECIAL_EFFECT_SEPIA,
	ISP_SPECIAL_EFFECT_NEGATIVE,
	ISP_SPECIAL_EFFECT_SOLARIZE,
	ISP_SPECIAL_EFFECT_POSTERIZE,
	ISP_SPECIAL_EFFECT_AQUA,
	ISP_SPECIAL_EFFECT_TOTAL
};

enum {
	ISP_CONTRAST_LV0, // -2
	ISP_CONTRAST_LV1, // -1.5
	ISP_CONTRAST_LV2, // -1
	ISP_CONTRAST_LV3, // -0.5
	ISP_CONTRAST_LV4, // 0
	ISP_CONTRAST_LV5, // 0.5
	ISP_CONTRAST_LV6, // 1
	ISP_CONTRAST_LV7, // 1.5
	ISP_CONTRAST_LV8, // 2
	// ISP_CONTRAST_NORMAL,
	// ISP_CONTRAST_LOW,
	// ISP_CONTRAST_HIGH,
	ISP_CONTRAST_TOTAL
};

enum isp_cfg_param {
	ISP_CFG_SET_ISO_SPEED,
	ISP_CFG_SET_AWB_GAIN,
	ISP_CFG_SET_DLD_SEQUENCE,
	ISP_CFG_SET_3A_CFG,
	ISP_CFG_SET_AE_CFG,
	ISP_CFG_SET_AF_CFG,
	ISP_CFG_SET_AWB_CFG,
	ISP_CFG_SET_YHIS_CFG,
	ISP_CFG_SET_SUB_SAMP_CFG,
	ISP_CFG_SET_AFL_CFG,
	ISP_CFG_SET_DLD_SEQ_BASIC_PREV,
	ISP_CFG_SET_DLD_SEQ_ADV_PREV,
	ISP_CFG_SET_DLD_SEQ_BASIC_FAST_CONV,
	ISP_CFG_SET_SCENARIO_INFO,
	ISP_CFG_SET_SHARPNESS,
	ISP_CFG_SET_SATURATION,
	ISP_CFG_SET_CONTRAST,
	ISP_CFG_SET_SPECIAL_EFFECT,
	// Altek update altek_isp_drv
	ISP_CFG_SET_TOTAL
};

enum isp_dev_capability {
	ISP_GET_FW_BUF_SIZE,
	ISP_GET_STATIS_BUF_SIZE,
	ISP_GET_DRAM_BUF_SIZE,
	ISP_GET_HIGH_ISO_BUF_SIZE,
	ISP_GET_CONTINUE_SIZE,
	ISP_GET_SINGLE_SIZE,
};

enum isp_dev_read_id {
	ISP_IMG_GET_STATISTICS_FRAME = 0,
	ISP_IMG_GET_FRAME,
};

enum isp_dev_write_id {
	ISP_IMG_STOP_ISP = 0,
};

typedef enum
{
    SENSOR_MODULE_TYPE_IMX214 = 0,
} SCINFO_SENSOR_MODULE_TYPE;

typedef enum
{
    COLOR_ORDER_RG = 0,
    COLOR_ORDER_GR,
    COLOR_ORDER_GB,
    COLOR_ORDER_BG
} SCINFO_COLOR_ORDER;

/*3A related parameters, provided by Altek_AP3AInfo.h*/
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint8_t u8;
typedef int8_t s8;


struct isp_awb_gain_info{
	uint16_t r;
	uint16_t g;
	uint16_t b;
};

/**
@typedef StatisticsDldRegion
@brief Statistics download region
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u16 uwBorderRatioX;
	u16 uwBorderRatioY;
	u16 uwBlkNumX;
	u16 uwBlkNumY;
	u16 uwOffsetRatioX;
	u16 uwOffsetRatioY;
}StatisticsDldRegion;
#pragma pack(pop)

/**
@typedef AE_CfgInfo
@brief AE configuration information
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u16 TokenID;
	StatisticsDldRegion tAERegion;
}AE_CfgInfo;
#pragma pack(pop)


/**
@structAWBHIS
@brief AWB histogram Parameters setting
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
    u8  bEnable;
    s8 cCrStart;
    s8 cCrEnd;
    s8 cOffsetUp;
    s8 cOffsetDown;
    s8 cCrPurple;
    u8 ucOffsetPurPle;
    s8 cGrassOffset;
    s8 cGrassStart;
    s8 cGrassEnd;
    u8 ucDampGrass;
    s8 cOffset_bbr_w_start;
    s8 cOffset_bbr_w_end;
    u8 ucYFac_w;
    u32 dHisInterp;
}AWBHIS;
#pragma pack(pop)


/**
@typedef AWB_CfgInfo
@brief AWB configuration information
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
#define AWB_UCYFACTOR_NUM              16
#define AWB_BBRFACTOR_NUM              33
typedef struct
{
	u16 TokenID;
	StatisticsDldRegion tAWBRegion;
	u8 ucYFactor[AWB_UCYFACTOR_NUM];
	s8 BBrFactor[AWB_BBRFACTOR_NUM];
	u16  uwRGain;
	u16  uwGGain;
	u16  uwBGain;
	u8   ucCrShift;
	u8   ucOffsetShift;
	u8   ucQuantize;
	u8   ucDamp;
	u8   ucSumShift;
	AWBHIS tHis;
    u16  uwRLinearGain;
    u16  uwBLinearGain;
}AWB_CfgInfo;
#pragma pack(pop)

/**
@typedef AFStatisticsDldRegion
@brief AF Statistics download region
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u16 uwSizeRatioX;       // used to control ROI of width, if set 50 means use half image to get HW3A stats data
	u16 uwSizeRatioY;       // used to control ROI of height, if set 50 means use half image to get HW3A stats data
	u16 uwBlkNumX;          // block number of horizontal direction of ROI
	u16 uwBlkNumY;          // block number of vertical direction of ROI
	u16 uwOffsetRatioX;    // used to control ROI shift position ratio of width, if set 1 means offset 1% width from start horizontal position, suggest 0
	u16 uwOffsetRatioY;    // used to control ROI shift position ratio of height, if set 1 means offset 1% width from start horizontal position, suggest 0
}AFStatisticsDldRegion;
#pragma pack(pop)


/**
@typedef MID_mode
@brief mid mode
*/
typedef enum
{
    MF_51_MODE,
    MF_31_MODE,
    MF_DISABLE,
}MID_MODE;

/**
@typedef AF_CfgInfo
@brief AF configuration information
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u16 TokenID;
	AFStatisticsDldRegion tAFRegion; // AF ROI configuration
	u8 bEnableAFLUT;    // Enable flag for tone mapping, set false would disable auwAFLUT
	u16 auwLUT[259];
	u16 auwAFLUT[259];
	u8 aucWeight[6];
	u16 uwSH;
	u8 ucThMode;
	u8 aucIndex[82];
	u16 auwTH[4];
	u16 pwTV[4];
	u32 udAFoffset;
	u8 bAF_PY_Enable;
	u8 bAF_LPF_Enable;
	MID_MODE nFilterMode;
	u8 ucFilterID;
	u16 uwLineCnt;
}AF_CfgInfo;
#pragma pack(pop)

/**
@typedef AF_CfgInfo
@brief AF configuration information
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u16 TokenID;
	StatisticsDldRegion tYHisRegion;
}YHis_CfgInfo;
#pragma pack(pop)

/**
@typedef AntiFlicker_CfgInfo
@brief AntiFlicker configuration information
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u16 TokenID;
	u16 uwOffsetRatioX;
	u16 uwOffsetRatioY;
}AntiFlicker_CfgInfo;
#pragma pack(pop)

/**
@typedef SubSample_CfgInfo
@brief SubSample configuration information
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u16 TokenID;
	u32 udBufferImageSize;
	u16 uwOffsetRatioX;
	u16 uwOffsetRatioY;
}SubSample_CfgInfo;
#pragma pack(pop)

/**
@typedef DldSequence
@brief Download sequence structure
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u8 ucPreview_Baisc_DldSeqLength;
	u8 ucPreview_Adv_DldSeqLength;
	u8 ucFastConverge_Baisc_DldSeqLength;
	u8 aucPreview_Baisc_DldSeq[MAXSEQLEN];
	u8 aucPreview_Adv_DldSeq[MAXSEQLEN];
	u8 aucFastConverge_Baisc_DldSeq[MAXSEQLEN];
}DldSequence;
#pragma pack(pop)

/**
@typedef Cfg3A_Info
@brief 3A configuration
*/
#pragma pack(push) /* push current alignment to stack */
#pragma pack(4) /* set alignment to 1 byte boundary */
typedef struct
{
	u32 ulMagicNum; //For sync between ISP firmware and 3A's lib
	AE_CfgInfo tAEInfo;
	AWB_CfgInfo tAWBInfo;
	AF_CfgInfo tAFInfo;
	YHis_CfgInfo tYHisInfo;
	AntiFlicker_CfgInfo tAntiFlickerInfo;
	SubSample_CfgInfo tSubSampleInfo;
}Cfg3A_Info;
#pragma pack(pop)

typedef struct
{
	u8                 ucSensorMode; /*FR,  binning_1, binning_2*/
	SCINFO_SENSOR_MODULE_TYPE   ucSensorMouduleType; /*IMX_219, OV4688*/
	u16                uwWidth; /*Width of Raw image*/
	u16                uwHeight; /*Height of Raw image*/
	u16                uwFrameRate; /* FPS*/
	u32                udLineTime; /*line time x100*/
	SCINFO_COLOR_ORDER  nColorOrder; /*color order*/
	u16                uwClampLevel; /*sensor's clamp level*/
} SCINFO_MODE_INFO_ISP;

typedef struct
{
	u8    bBypassLV;
	u8    bBypassVideo;
	u8    bBypassStill;
	u8    bBypassMetaData;
} SCINFO_OUT_BYPASSFLG;

typedef struct
{
	u16     uwBayerSCLOutWidth;
	u16     uwBayerSCLOutHeight;
} SCINFO_BAYERSCL_OUT_INFO;

typedef struct
{
	SCINFO_MODE_INFO_ISP    tSensorInfo; /* tModeInfo */
	SCINFO_OUT_BYPASSFLG    tScenarioOutBypassFlag;
	SCINFO_BAYERSCL_OUT_INFO tBayerSCLOutInfo;
} SCENARIO_INFO_AP;

struct sprd_isp_time {
	uint32_t sec;
	uint32_t usec;
};

/*After one statistics buffer has been used by 3A library.
**user need return this buffer to kernel.
*/
struct isp_statis_buf {
	uint32_t                         buf_size;
	unsigned long                    phy_addr;
	unsigned long                    vir_addr;
};

struct isp_cfg_img_buf {
	uint32_t                         format;
	uint32_t                         img_id;
	uint32_t                         width;
	uint32_t                         height;
	unsigned long                    yaddr;
	unsigned long                    uaddr;
	unsigned long                    vaddr;
	unsigned long                    yaddr_vir;
	unsigned long                    uaddr_vir;
	unsigned long                    vaddr_vir;
};

struct isp_statis_frame_output {
	uint32_t                         format;
	uint32_t                         buf_size;
	unsigned long                    phy_addr;
	unsigned long                    vir_addr;
	struct sprd_isp_time             time_stamp;
};

struct isp_statis_frame {
	uint32_t                         format;
	uint32_t                         evt;
	uint32_t                         buf_size;
	unsigned long                    phy_addr;
	unsigned long                    vir_addr;
	struct sprd_isp_time             time_stamp;
};

struct isp_addr {
	unsigned long                    chn0;
	unsigned long                    chn1;
	unsigned long                    chn2;
};

struct isp_cfg_img_param {
	uint32_t                    img_id;//0-preview, 1-video, 2-still capture 3-statistics
	uint32_t                    dram_eb;
	uint32_t                    format;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    buf_num;
	struct isp_addr      addr[IMG_BUF_NUM_MAX];
	struct isp_addr      addr_vir[IMG_BUF_NUM_MAX];
	int32_t                      addr_mfd[IMG_BUF_NUM_MAX];//iommu fd
	uint32_t                    line_offset;
};

struct isp_img_frame_output {
	uint32_t                    format;
	uint32_t                    fid;
	uint32_t                    img_id;
	uint32_t                    irq_id;
	uint32_t                    width;
	uint32_t                    height;
	uint32_t                    yaddr;
	uint32_t                    uaddr;
	uint32_t                    vaddr;
	uint32_t                    yaddr_vir;
	uint32_t                    uaddr_vir;
	uint32_t                    vaddr_vir;
	struct sprd_isp_time        time_stamp;
};

struct isp_img_read_op {
	uint32_t cmd;
	uint32_t evt;
	union {
		struct isp_statis_frame_output    statis_frame;
		struct isp_img_frame_output       img_frame;
	} param;
};

struct isp_img_write_op {
	uint32_t cmd;
	uint32_t reserved;
};

struct isp_img_size {
	uint32_t width;
	uint32_t height;
};

struct isp_init_mem_param {
	uint32_t                          fw_buf_size;
	signed int                        fw_buf_mfd;
	unsigned long                     fw_buf_vir_addr;
	unsigned long long                     fw_buf_phy_addr;//full mode channel
	uint32_t                          dram_buf_size;
	unsigned long                     dram_buf_vir_addr;
	unsigned long                     dram_buf_phy_addr;//full mode channel
	uint32_t                          high_iso_buf_size;
	unsigned long                     high_iso_buf_vir_addr;
	unsigned long                     high_iso_phy_addr;//full mode channel
};

//TBD
struct isp_dev_init_param {
	uint32_t                     camera_id;
	uint32_t                     width;
	uint32_t                     height;
};

struct isp_io_param {
//	uint32_t                     sensor_id;
	uint32_t                     sub_id;
	void *                       property_param;
	uint32_t                     reserved;
};

struct isp_capability {
//	uint32_t isp_id;
	uint32_t                     index;
	void *                       property_param;
};

struct isp_irq {
	uint32_t                     irq_val0;
	uint32_t                     reserved;
	int32_t                      ret_val;
	struct sprd_isp_time         time_stamp;
};

#define ISP_IO_MAGIC               'R'
#define ISP_IO_LOAD_FW             _IOW(ISP_IO_MAGIC, 0, struct isp_init_mem_param)
#define ISP_IO_IRQ                 _IOR(ISP_IO_MAGIC, 1, struct isp_irq)
#define ISP_IO_SET_STATIS_BUF      _IOW(ISP_IO_MAGIC, 2, struct isp_statis_buf)
#define ISP_IO_SET_IMG_BUF         _IOW(ISP_IO_MAGIC, 3, struct isp_cfg_img_buf)
#define ISP_IO_SET_IMG_PARAM       _IOW(ISP_IO_MAGIC, 4, struct isp_cfg_img_param)
#define ISP_IO_STREAM_ON           _IOW(ISP_IO_MAGIC, 5, uint32_t)
#define ISP_IO_STREAM_OFF          _IOW(ISP_IO_MAGIC, 6, uint32_t)
#define ISP_IO_SET_INIT_PARAM      _IOW(ISP_IO_MAGIC, 7, struct isp_dev_init_param)
#define ISP_IO_STOP                _IOW(ISP_IO_MAGIC, 8, uint32_t)
#define ISP_IO_CAPABILITY          _IOR(ISP_IO_MAGIC, 9, struct isp_capability)
#define ISP_IO_CFG_PARAM           _IOWR(ISP_IO_MAGIC, 10, struct isp_io_param)
#define ISP_IO_GET_TIME            _IOR(ISP_IO_MAGIC, 11, struct sprd_isp_time)
#define ISP_IO_GET_STATIS_BUF      _IOR(ISP_IO_MAGIC, 12, struct isp_statis_frame)
#define ISP_IO_SET_DCAM_ID             _IOR(ISP_IO_MAGIC, 13, uint32_t)

#endif
